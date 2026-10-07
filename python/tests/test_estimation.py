# SPDX-License-Identifier: Apache-2.0
"""am_sdk.estimation: calibration record, current model, quality flags, collision observer, UKF,
learned residual hooks and the simulator harness. SIMULATED values only; the bounds asserted here are
regression limits for the simulator, not product accuracy."""
import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import am  # noqa: E402
from am_sdk import estimation as E  # noqa: E402
from am_sdk.estimation import simharness as H  # noqa: E402

try:
    import numpy  # noqa: F401
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


def exact_plant(cal):
    """TruthPlant equal to the calibration (no spread, no sensor errors)."""
    return am.TruthPlant(kt_scale=1.0, kt_ref_C=cal.kt_ref_temp_C, kt_temp_coeff_per_K=cal.kt_temp_coeff_per_K,
                         coulomb_Nm=cal.coulomb_Nm, breakaway_Nm=cal.breakaway_Nm,
                         stribeck_vel_rad_s=cal.stribeck_vel_rad_s, viscous_Nm_s_per_rad=cal.viscous_Nm_s_per_rad,
                         friction_ref_temp_C=cal.friction_ref_temp_C,
                         coulomb_temp_coeff_per_K=cal.coulomb_temp_coeff_per_K,
                         viscous_temp_exp_per_K=cal.viscous_temp_exp_per_K, eta_forward=cal.eta_forward,
                         eta_reverse=cal.eta_reverse, bearing_Nm=cal.bearing_Nm)


def sim(unit, plant, **opts):
    o = am.SimOptions(truth_plant=plant, **opts)
    s = am.am_init(am.AmSessionConfig(interface="sim:" + unit, cycle_us=1000, sim_options=[o]))
    j = am.am_open_joint(s, 0)
    am.am_start(s)
    am.am_set_mode(j, am.AmMode.IMPEDANCE)
    am.am_enable(j, 1000)
    return s, j


class Calibration(unittest.TestCase):
    def test_design_records_for_every_unit_are_valid_and_round_trip(self):
        for u in am.UNIT_IDS:
            cal = E.EstimatorCalibration.from_unit(u)
            self.assertEqual(cal.validate(), [], u)
            self.assertEqual(cal.data_basis, "design_target")
            back = E.EstimatorCalibration.from_json(cal.to_json())
            self.assertEqual(back, cal)

    def test_schema_matches_the_dataclass(self):
        schema = E.load_schema()
        self.assertEqual(schema["properties"]["schema"]["const"], E.SCHEMA_ID)
        names = {f for f in E.EstimatorCalibration.__dataclass_fields__}
        self.assertEqual(set(schema["properties"]), names)
        for r in schema["required"]:
            self.assertIn(r, names)
        json.dumps(schema)

    def test_validation_catches_bad_records(self):
        d = E.EstimatorCalibration.from_unit("M").to_dict()
        for k, v in (("coulomb_Nm", 99.0), ("eta_forward", 1.2), ("gear_ratio", 0.0), ("schema", "x"),
                     ("friction_map", [[1.0, 0.5], [0.5, 0.6]]), ("kt_Nm_per_A", float("nan"))):
            bad = dict(d)
            bad[k] = v
            self.assertTrue(E.validate_dict(bad), k)
            with self.assertRaises(ValueError):
                E.EstimatorCalibration.from_dict(bad)
        bad = dict(d)
        bad["efficiency_map"] = {"temps_C": [20, 60], "loads_Nm": [0, 10], "eta_forward": [[0.9, 0.9]],
                                 "eta_reverse": [[0.8, 0.8], [0.8, 0.8]]}
        self.assertTrue(E.validate_dict(bad))

    def test_from_joint_on_the_simulator(self):
        s, j = sim("H-F", None)
        cal = E.EstimatorCalibration.from_joint(j)
        self.assertEqual(cal.unit, "H-F")
        self.assertAlmostEqual(cal.gear_ratio, j.jb.ud.ratio)


class CurrentModel(unittest.TestCase):
    def setUp(self):
        self.cal = H.study_calibration("S-S")

    def forward_iq(self, tau_t, w, T):
        """Steady-state current the rigid-gear model needs to deliver tau_t at the gear output."""
        cal = self.cal
        g = 1.0 / cal.eta_forward if tau_t * w > 0 else cal.eta_reverse
        tm = E.friction_at(cal, w, T) + g * tau_t
        return tm / (cal.gear_ratio * E.kt_at(cal, T, 0.0))

    def test_inverts_the_forward_model_while_moving(self):
        cal = self.cal
        for w in (-3.0, -0.5, 0.4, 2.0):
            for tau_t in (-12.0, -3.0, 2.0, 10.0):
                iq = self.forward_iq(tau_t, w, 50.0)
                e = E.current_model(cal, iq, w, 0.0, 50.0)
                want = tau_t - cal.bearing_Nm * math.tanh(w / 0.01)
                self.assertAlmostEqual(e.tau_out_Nm, want, delta=1e-3 * abs(tau_t) + 1e-6, msg=(w, tau_t))
                self.assertEqual(e.quality, E.EST_OK)
                self.assertEqual(e.label, "estimated")
                self.assertGreater(e.bound_Nm, 0.0)

    def test_stick_is_flagged_with_the_stick_band(self):
        cal = self.cal
        e = E.current_model(cal, 1.0, 0.0, 0.0, 40.0)
        self.assertTrue(e.quality & E.EST_STICK)
        tm = cal.gear_ratio * E.kt_at(cal, 40.0, 1.0)
        band = E.stick_band_Nm(cal, tm, 40.0)
        self.assertGreaterEqual(e.bound_Nm, band)
        lo, hi = (tm - cal.breakaway_Nm) * cal.eta_forward, (tm + cal.breakaway_Nm) / cal.eta_reverse
        self.assertAlmostEqual(band, 0.5 * (hi - lo), places=9)
        self.assertLess(e.confidence, 0.5)
        self.assertEqual(E.quality_str(e.quality), "stick")

    def test_temperature_correction_and_missing_temperature(self):
        cal = self.cal
        hot = E.current_model(cal, 2.0, 1.0, 0.0, 100.0)
        cold = E.current_model(cal, 2.0, 1.0, 0.0, 20.0)
        self.assertLess(hot.tau_motor_Nm, cold.tau_motor_Nm)       # Kt falls with magnet temperature
        self.assertAlmostEqual(hot.tau_motor_Nm / cold.tau_motor_Nm,
                               (1 + cal.kt_temp_coeff_per_K * 40) / (1 - cal.kt_temp_coeff_per_K * 40), places=12)
        none = E.current_model(cal, 2.0, 1.0, 0.0, None)
        self.assertTrue(none.quality & E.EST_NO_TEMPERATURE)
        ref = E.current_model(cal, 2.0, 1.0, 0.0, cal.kt_ref_temp_C)
        self.assertAlmostEqual(none.tau_out_Nm, ref.tau_out_Nm, places=12)
        self.assertGreater(none.bound_Nm, ref.bound_Nm)

    def test_saturation_and_maps(self):
        cal = H.study_calibration("S-S")
        cal.kt_sat_frac, cal.kt_sat_current_A = 0.05, 20.0
        e = E.current_model(cal, 25.0, 1.0, 0.0, 60.0)
        self.assertTrue(e.quality & E.EST_SATURATED)
        self.assertAlmostEqual(E.kt_at(cal, 60.0, 20.0), cal.kt_Nm_per_A * 0.95, places=12)
        cal.efficiency_map = E.EfficiencyMap(temps_C=[20.0, 80.0], loads_Nm=[0.0, 20.0],
                                             eta_forward=[[0.90, 0.94], [0.92, 0.96]],
                                             eta_reverse=[[0.80, 0.85], [0.82, 0.87]])
        self.assertEqual(cal.validate(), [])
        self.assertEqual(E.efficiency_at(cal, 20.0, 0.0), (0.90, 0.80))
        ef, er = E.efficiency_at(cal, 50.0, 10.0)
        self.assertAlmostEqual(ef, 0.93)
        self.assertAlmostEqual(er, 0.835)
        self.assertEqual(E.efficiency_at(cal, 200.0, -99.0), (0.96, 0.87))   # clamped, |load|
        cal.friction_map = [[0.0, 0.6], [1.0, 0.5], [5.0, 0.9]]
        self.assertAlmostEqual(E.friction_at(cal, 3.0, cal.friction_ref_temp_C), 0.7, places=6)
        self.assertAlmostEqual(E.friction_at(cal, -3.0, cal.friction_ref_temp_C), -0.7, places=6)

    def test_dither_is_bounded_and_flagged(self):
        cal = self.cal
        amax = E.dither_amplitude_max(cal)
        self.assertAlmostEqual(amax, min(1.5 * cal.breakaway_Nm, 0.1 * cal.rated_torque_Nm))
        self.assertLessEqual(max(abs(E.dither_torque(cal, 100.0, 60.0, k * 1e-4)) for k in range(200)), amax + 1e-12)
        e = E.current_model(cal, 1.0, 0.0, 0.0, 40.0, dither=True)
        self.assertTrue(e.quality & E.EST_DITHER)
        self.assertFalse(e.quality & E.EST_STICK)


class Estimator(unittest.TestCase):
    def test_tracks_an_exactly_calibrated_simulated_joint(self):
        cal = H.study_calibration("S-S")
        P = H.STUDY_S["S-S"]
        grav = (lambda t, q, qd: -P["mgl"] * math.cos(q))
        s, j = sim("S-S", exact_plant(cal), load_inertia_kgm2=P["J_L"], external_torque=grav, ambient_C=40.0)
        h = H.EstimatorHarness(s, j, E.TorqueEstimator(cal))
        for k in range(3000):
            t = k * 1e-3
            q = 0.6 * math.sin(2 * math.pi * 0.5 * t)
            qd = 0.6 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t)
            am.am_set_impedance(j, am.AmImpedance(q_ref_rad=q, qd_ref_rad_s=qd, kp_Nm_per_rad=150.0,
                                                  kd_Nm_s_per_rad=4.0, tau_ff_Nm=P["mgl"] * math.cos(q)))
            h.step()
        log = h.log[300:]
        tr = H.lp1([x.truth_tau_out_Nm for x in log], 30.0, 1e-3)
        es = H.lp1([x.est.tau_out_Nm for x in log], 30.0, 1e-3)
        mov = [i for i, x in enumerate(log) if abs(x.est.qd_rad_s) > 0.2]
        st = H.error_stats([es[i] - tr[i] for i in mov], cal.rated_torque_Nm)
        self.assertLess(st["p95_pct_rated"], 3.0, st)          # simulator regression limit, not a claim
        stick_n = sum(1 for x in log if x.est.quality & E.EST_STICK)
        self.assertGreater(stick_n, 0)                          # reversals pass through stick

    def test_reversal_hold_and_invalid_inputs(self):
        cal = H.study_calibration("S-D")
        est = E.TorqueEstimator(cal)
        e = est.update(1e-3, 0.0, 0.5, 40.0)
        self.assertTrue(e.quality & E.EST_STICK)
        q = 0.0
        flagged = []
        for k in range(100):
            q += 0.5e-3                                          # 0.5 rad/s
            flagged.append(est.update(1e-3, q, 0.5, 40.0).quality)
        first_moving = next(i for i, f in enumerate(flagged) if not f & E.EST_STICK)
        self.assertTrue(flagged[first_moving] & E.EST_AMBIGUOUS)
        self.assertEqual(flagged[-1], E.EST_OK)
        bad = est.update(1e-3, float("nan"), 0.5, 40.0)
        self.assertEqual(bad.quality, E.EST_INVALID)
        self.assertEqual(bad.confidence, 0.0)
        self.assertTrue(est.update(1e-3, q, 0.5, 40.0, valid=False).quality & E.EST_INVALID)

    def test_telemetry_without_extended_fields_has_no_temperature(self):
        cfg = am.AmSessionConfig(interface="sim:M", telemetry_extended=False)
        s = am.am_init(cfg)
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        am.am_cycle(s)
        est = E.TorqueEstimator(E.EstimatorCalibration.from_unit("M"))
        e = est.update_from_telemetry(am.am_read_telemetry(j))
        self.assertTrue(e.quality & E.EST_NO_TEMPERATURE)


class Collision(unittest.TestCase):
    def test_observer_follows_a_contact_step_and_releases(self):
        J, dt = 0.07, 1e-3
        cfg = E.CollisionConfig(inertia_kgm2=J, gain_hz=30.0, threshold_Nm=1.0, min_hold_s=0.02)
        det = E.CollisionDetector(cfg)
        qd = 1.0
        onsets, rs = [], []
        for k in range(600):
            contact = -3.0 if 200 <= k < 300 else 0.0
            drive = 0.5
            qd += dt * (drive + contact) / J
            o = det.update(dt, qd, drive, 0.0, 0)
            rs.append(o.residual_Nm)
            if o.onset:
                onsets.append(k)
        self.assertEqual(len(onsets), 1)
        self.assertLessEqual(onsets[0] - 200, 3)
        self.assertAlmostEqual(rs[299], -3.0, delta=0.01)      # first-order lag settles on the contact torque
        self.assertLess(abs(rs[199]), 1e-9)
        self.assertFalse(det.active)
        self.assertEqual(len(det.events), 1)

    def test_threshold_scaled_while_stuck_and_calibration(self):
        det = E.CollisionDetector(E.CollisionConfig(inertia_kgm2=0.1, threshold_Nm=2.0, stick_scale=2.0))
        o = det.update(1e-3, 0.0, 0.0, 0.0, E.EST_STICK)
        self.assertEqual(o.threshold_Nm, 4.0)
        o = det.update(1e-3, 0.0, 0.0, 0.0, 0)
        self.assertEqual(o.threshold_Nm, 2.0)
        self.assertAlmostEqual(E.calibrate_threshold([0.1, -0.5, float("nan"), 0.3]), 0.55)

    def test_study_collision_case_in_the_simulator(self):
        """Shortened S_torque_sensing_R1 collision case: threshold from contact-free motion, then a wall."""
        unit = "S-S"
        P = H.STUDY_S[unit]
        cal = H.study_calibration(unit)
        plant = H.study_truth_plant(unit)
        J = cal.reflected_inertia_kgm2 + P["J_L"]

        def grav(t, q, qd):
            return -P["mgl"] * math.cos(q)

        def run(ref, q0, n, wall_k=0.0, th=math.inf):
            def contact(t, q, qd):
                return min(0.0, -wall_k * (q - 0.4) - 5.0 * qd) if (wall_k and q > 0.4) else 0.0
            s, j = sim(unit, plant, load_inertia_kgm2=P["J_L"], q0_rad=q0, ambient_C=45.0,
                       external_torque=lambda t, q, qd: grav(t, q, qd) + contact(t, q, qd))
            det = E.CollisionDetector(E.CollisionConfig(inertia_kgm2=J, threshold_Nm=th))
            h = H.EstimatorHarness(s, j, E.TorqueEstimator(cal), det, known_torque=grav, contact_torque=contact)
            for k in range(n):
                q, qd, qdd = ref(k * 1e-3)
                am.am_set_impedance(j, am.AmImpedance(q_ref_rad=q, qd_ref_rad_s=qd, kp_Nm_per_rad=P["Kp"],
                                                      kd_Nm_s_per_rad=P["Kd"],
                                                      tau_ff_Nm=P["mgl"] * math.cos(q) + J * qdd))
                h.step()
            return h.log

        def sine(t):
            w = 2 * math.pi * 0.8
            return 0.5 * math.sin(w * t), 0.5 * w * math.cos(w * t), -0.5 * w * w * math.sin(w * t)

        def approach(t):
            if t < 0.15:
                return -0.3 + 0.5 * 2.0 / 0.15 * t * t, 2.0 * t / 0.15, 2.0 / 0.15
            return -0.3 + 0.15 + 2.0 * (t - 0.15), 2.0, 0.0
        free = run(sine, 0.0, 2500)
        th = E.calibrate_threshold([x.residual_Nm for x in free[200:]])
        self.assertLess(th, 0.5 * cal.rated_torque_Nm)
        log = run(approach, -0.3, 700, wall_k=3000.0, th=th)
        d = H.detection_latency(log)
        self.assertTrue(d["detected"], d)
        self.assertLessEqual(d["latency_s"], 0.010, d)          # simulator regression limit
        self.assertEqual(d["false_alarms_before_contact"], 0)


class Ukf(unittest.TestCase):
    def test_ukf_runs_and_stays_bounded(self):
        cal = H.study_calibration("S-D")
        ukf = E.UkfTorqueEstimator(cal)
        base = E.TorqueEstimator(cal)
        q = 0.0
        for k in range(1500):
            t = k * 1e-3
            qd = 1.5 * math.cos(2 * math.pi * 0.7 * t)
            q += qd * 1e-3
            tau_t = 2.0 * math.sin(2 * math.pi * 0.3 * t)
            g = 1 / cal.eta_forward if tau_t * qd > 0 else cal.eta_reverse
            iq = (E.friction_at(cal, qd, 40.0) + g * tau_t) / (cal.gear_ratio * E.kt_at(cal, 40.0, 0.0))
            u = ukf.update(1e-3, q, iq, 40.0)
            b = base.update(1e-3, q, iq, 40.0)
            self.assertTrue(math.isfinite(u.tau_out_Nm))
        self.assertLess(abs(u.tau_out_Nm - b.tau_out_Nm), 0.3 * cal.rated_torque_Nm)
        self.assertGreater(u.bound_Nm, 0.0)


class LearnedResidual(unittest.TestCase):
    def dyno_points(self, cal, extra):
        pts = []
        for i in range(400):
            w = (0.05 + 4.0 * ((i * 37) % 100) / 100.0) * (1 if i % 2 else -1)
            tau = -10.0 + 20.0 * ((i * 53) % 97) / 96.0
            T = 25.0 + 60.0 * ((i * 11) % 13) / 12.0
            tm = E.baseline_loss(cal, w, tau + cal.bearing_Nm * math.tanh(w / 0.01), T) + extra(w, tau, T)
            iq = tm / (cal.gear_ratio * E.kt_at(cal, T, 0.0))
            pts.append((w, tau, T, iq))
        return pts

    def test_linear_residual_is_adopted_only_when_it_helps(self):
        cal = H.study_calibration("S-S")
        extra = (lambda w, tau, T: 0.2 * math.tanh(w / 0.003) + 0.01 * w * (T - 40.0))
        samples = E.residual_targets(cal, self.dyno_points(cal, extra))
        res = E.LinearFeatureResidual()
        fit_rmse = res.fit(samples[:300])
        self.assertLess(fit_rmse, 0.02)
        chk = E.adoption_check(res, samples[300:])
        self.assertTrue(chk["adopt"], chk)
        none = E.residual_targets(cal, self.dyno_points(cal, lambda w, tau, T: 0.0))
        self.assertFalse(E.adoption_check(res, none[300:])["adopt"])
        back = E.LinearFeatureResidual.from_dict(json.loads(json.dumps(res.to_dict())))
        self.assertAlmostEqual(back(1.0, 2.0, 50.0, 1.0), res(1.0, 2.0, 50.0, 1.0))
        ukf = E.UkfTorqueEstimator(cal, residual=res)
        self.assertTrue(math.isfinite(ukf.update(1e-3, 0.0, 1.0, 40.0).tau_out_Nm))

    @unittest.skipUnless(HAVE_NUMPY, "numpy not installed (extra [pinn])")
    def test_pinn_residual_trains(self):
        from am_sdk.estimation.pinn import PinnResidual
        cal = H.study_calibration("S-S")
        extra = (lambda w, tau, T: 0.15 * math.tanh(w / 0.003) * math.exp(-abs(w)) + 0.004 * abs(tau) * math.tanh(w / 0.003))
        samples = E.residual_targets(cal, self.dyno_points(cal, extra))
        p = PinnResidual(hidden=12, seed=1)
        rmse = p.fit(samples[:300], cal=cal, epochs=600)
        self.assertLess(rmse, 0.05)
        chk = E.adoption_check(p, samples[300:])
        self.assertTrue(chk["adopt"], chk)
        back = PinnResidual.from_dict(json.loads(json.dumps(p.to_dict())))
        self.assertAlmostEqual(back(1.0, 2.0, 50.0, 1.0), p(1.0, 2.0, 50.0, 1.0), places=12)


class SimHook(unittest.TestCase):
    def test_default_plant_truth_and_unchanged_motion(self):
        """Without a truth plant the sim moves exactly as before; truth = J_load * a - external."""
        ext = (lambda t, q, qd: -2.0 * math.cos(q))
        o = am.SimOptions(load_inertia_kgm2=0.1, external_torque=ext)
        s1 = am.am_init(am.AmSessionConfig(interface="sim:M", sim_options=[o]))
        j1 = am.am_open_joint(s1, 0)
        am.am_start(s1)
        am.am_set_mode(j1, am.AmMode.CST)
        am.am_enable(j1, 1000)
        for k in range(200):
            am.am_set_torque(j1, 5.0)
            am.am_cycle(s1)
            tr = j1.jb.truth
            self.assertAlmostEqual(tr.tau_out_Nm, 0.1 * tr.qdd_rad_s2 - tr.tau_ext_Nm, places=9)
        self.assertGreater(j1.jb.q, 0.0)

    def test_truth_plant_reports_sensor_errors(self):
        plant = am.TruthPlant(i_offset_A=0.2, temp_sensor_bias_C=3.0, eta_forward=0.9, eta_reverse=0.8,
                              coulomb_Nm=0.5, breakaway_Nm=0.6)
        s, j = sim("M", plant, ambient_C=30.0)
        am.am_cycle(s)
        t = am.am_read_telemetry(j)
        self.assertAlmostEqual(t.iq_A, 0.2, places=3)
        self.assertAlmostEqual(t.t_magnet_C - j.jb.t_magnet, 3.0, places=9)


if __name__ == "__main__":
    unittest.main()
