# SPDX-License-Identifier: Apache-2.0
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import am, make  # noqa: E402

TWO_PI = 2 * math.pi
HEAVY = 1.0e4   # kg m^2 load that pins the joint near stall


# M published values (am_sdk/data/M/M_params.json)
M_JOUT = 0.030
M_BACKDRIVE = 1.4

class Limits(unittest.TestCase):
    def test_torque_never_above_published_peak_envelope(self):
        for unit in am.UNIT_IDS:
            s, j, sj = make(unit, load_inertia_kgm2=0.2)
            ud = sj.ud
            prev_w = 0.0
            for k in range(3000):
                am.am_set_torque(j, 10.0 * ud.peak_nm)       # far above anything allowed
                am.am_cycle(s)
                t = am.am_read_telemetry(j)
                env = ud.model.torque_limit(abs(prev_w), '48', 'peak') or 0.0
                self.assertLessEqual(abs(sj.torque_applied), env * 1.0001 + 1e-9, "%s cycle %d" % (unit, k))
                self.assertLessEqual(abs(sj.torque_applied), ud.peak_nm * 1.0001)
                prev_w = t.qd_rad_s
            self.assertTrue(t.internal_limit)                # clamping is visible

    def test_peak_lasts_two_seconds_then_continuous_envelope(self):
        s, j, sj = make("M", load_inertia_kgm2=HEAVY)
        ud = sj.ud
        taus = []
        for k in range(4000):
            am.am_set_torque(j, 500.0)
            am.am_cycle(s)
            taus.append(sj.torque_applied)
        self.assertAlmostEqual(taus[500], ud.peak_nm, places=3)
        self.assertAlmostEqual(taus[1900], ud.peak_nm, places=3)
        n_peak = sum(1 for t in taus[:2600] if t > 40.0)
        self.assertAlmostEqual(n_peak / 1000.0, 2.0, delta=0.05)         # 2 s of peak
        cont = ud.model.torque_limit(abs(sj.qd), '48', 'continuous')
        self.assertLessEqual(max(taus[2200:]), cont * 1.0001)
        self.assertGreater(min(taus[2200:3500]), 0.9 * cont)             # not shut off, held at continuous
        self.assertTrue(am.am_read_telemetry(j).warning_word & am.AM_WARN_PEAK_BUDGET_EMPTY)

    def test_peak_budget_recovers(self):
        s, j, sj = make("M", load_inertia_kgm2=HEAVY)
        for _ in range(3000):
            am.am_set_torque(j, 500.0)
            am.am_cycle(s)
        self.assertLess(sj.budget, 1.0)
        for _ in range(12000):                           # 12 s at zero torque
            am.am_set_torque(j, 0.0)
            am.am_cycle(s)
        am.am_set_torque(j, 500.0)
        am.am_cycle(s)
        self.assertGreater(sj.torque_applied, 40.0)      # peak available again

    def test_thermal_derate_engages(self):
        # H-X continuous curve (131.7 Nm at stall) sits above the 125.43 Nm thermal rating, so a
        # long hold at the curve must heat past x = 1 and the drive must derate.
        s, j, sj = make("H-X", cycle_us=20000, load_inertia_kgm2=HEAVY)
        sj.set_external_torque(lambda t, q, qd: -sj.tau_applied)       # lock the output at stall
        ud = sj.ud
        for k in range(30000):                           # 600 s
            am.am_set_torque(j, 500.0)
            am.am_cycle(s)
        t = am.am_read_telemetry(j)
        self.assertGreater(sj.thermal.x, 1.0)
        self.assertTrue(t.warning_word & am.AM_WARN_THERMAL_DERATE)
        cont_curve = ud.model.torque_limit(0.0, '48', 'continuous')
        self.assertLess(sj.torque_applied, cont_curve * 0.999)
        self.assertTrue(t.internal_limit)
        self.assertLess(sj.thermal.x, 1.12)              # derate pulls heating back toward x = 1
        self.assertGreater(t.t_winding_C, 90.0)
        self.assertEqual(t.state, am.AmDriveState.OPERATION_ENABLED)

    def test_derate_at_warm_start_and_ambient(self):
        s, j, sj = make("M", load_inertia_kgm2=HEAVY, thermal_x0=1.2)
        am.am_set_torque(j, 500.0)
        am.am_cycle(s)
        self.assertLess(sj.torque_applied, 36.3 / math.sqrt(1.2) * 1.001)
        self.assertTrue(am.am_read_telemetry(j).warning_word & am.AM_WARN_THERMAL_DERATE)
        # hot ambient lowers the derate threshold: temperature reading follows the model
        s2, j2, sj2 = make("M", ambient_C=40.0, thermal_x0=0.5)
        am.am_cycle(s2)
        self.assertAlmostEqual(am.am_read_telemetry(j2).t_winding_C, 40.0 + 80.0 * 0.5, delta=0.5)

    def test_over_temperature_trip_and_clear(self):
        s, j, sj = make("M", enable=False, cycle_us=20000, thermal_x0=1.5)
        for _ in range(3):
            am.am_cycle(s)
        t = am.am_read_telemetry(j)
        self.assertEqual(t.state, am.AmDriveState.FAULT)
        self.assertIn(am.FAULT_MOTOR_OVERTEMP, am.am_get_faults(j).codes)
        with self.assertRaises(am.AmError) as cm:
            am.am_enable(j, 100)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_FAULT)
        with self.assertRaises(am.AmError) as cm:
            am.am_clear_faults(j, 100)                  # still too hot
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_FAULT)
        for _ in range(3000):                            # cool down for 60 s
            am.am_cycle(s)
        am.am_clear_faults(j, 500)
        self.assertEqual(am.am_read_telemetry(j).state, am.AmDriveState.SWITCH_ON_DISABLED)
        am.am_enable(j, 2000)
        self.assertEqual(am.am_read_telemetry(j).state, am.AmDriveState.OPERATION_ENABLED)

    def test_speed_beyond_no_load_gives_no_motoring_torque(self):
        s, j, sj = make("M", load_inertia_kgm2=0.0)
        sj.qd = sj.ud.no_load_rad_s["48"] * 1.5
        sj.qd_f = sj.qd
        am.am_set_torque(j, 50.0)
        am.am_cycle(s)
        self.assertEqual(sj.torque_applied, 0.0)

    def test_customer_limits_lower_only(self):
        s, j, sj = make("M", enable=False, load_inertia_kgm2=HEAVY)
        eff, ceil = am.am_get_limits(j)
        bad = am.AmLimits(**{**eff.__dict__, "torque_pos_Nm": ceil.torque_pos_Nm * 1.01})
        with self.assertRaises(am.AmError) as cm:
            am.am_set_limits(j, bad)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_LIMIT)
        self.assertEqual(am.am_get_limits(j)[0].torque_pos_Nm, eff.torque_pos_Nm)      # nothing written
        ok = am.AmLimits(**{**eff.__dict__, "torque_pos_Nm": 20.0, "torque_neg_Nm": 15.0})
        am.am_set_limits(j, ok)
        am.am_enable(j, 1000)
        am.am_set_torque(j, 500.0)
        am.am_cycle(s)
        self.assertAlmostEqual(sj.torque_applied, 20.0, places=6)
        am.am_set_torque(j, -500.0)
        am.am_cycle(s)
        self.assertAlmostEqual(sj.torque_applied, -15.0, places=6)
        with self.assertRaises(am.AmError) as cm:
            am.am_set_limits(j, ok)                      # not while enabled
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STATE)

    def test_impedance_gain_envelope(self):
        s, j, sj = make("M", mode=am.AmMode.IMPEDANCE)
        kmax = am.am_get_limits(j)[0].kp_max_Nm_per_rad
        with self.assertRaises(am.AmError) as cm:
            am.am_set_impedance(j, am.AmImpedance(kp_Nm_per_rad=kmax * 2, kd_Nm_s_per_rad=1.0))
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_LIMIT)
        with self.assertRaises(am.AmError) as cm:
            am.am_set_impedance(j, am.AmImpedance(kp_Nm_per_rad=float("nan")))
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_PARAM)


class Dynamics(unittest.TestCase):
    def test_impedance_step_response(self):
        J = 0.1
        K = 100.0
        D = 2 * math.sqrt(K * (M_JOUT + J))
        s, j, sj = make("M", mode=am.AmMode.IMPEDANCE, load_inertia_kgm2=J, friction_scale=0.0)
        qs, step = [], 0.2
        for _ in range(3000):
            am.am_set_impedance(j, am.AmImpedance(q_ref_rad=step, kp_Nm_per_rad=K, kd_Nm_s_per_rad=D))
            am.am_cycle(s)
            qs.append(am.am_read_telemetry(j).q_rad)
        self.assertLess(max(qs) - step, 0.1 * step)                     # overshoot below 10 %
        self.assertAlmostEqual(qs[-1], step, delta=0.002)                # settles on target
        t63 = next(i for i, q in enumerate(qs) if q >= 0.632 * step) * 1e-3
        wn = math.sqrt(K / (M_JOUT + J))
        self.assertTrue(0.3 / wn < t63 < 3.0 / wn, t63)                  # rise time on the order of 1/wn

    def test_backdrive_friction_sets_steady_state_error(self):
        s, j, sj = make("M", mode=am.AmMode.IMPEDANCE)
        K = 100.0
        for _ in range(2000):
            am.am_set_impedance(j, am.AmImpedance(q_ref_rad=0.5, kp_Nm_per_rad=K, kd_Nm_s_per_rad=3.0))
            am.am_cycle(s)
        err = 0.5 - am.am_read_telemetry(j).q_rad
        self.assertAlmostEqual(err * K, sj.ud.backdrive_nm, delta=0.05 * sj.ud.backdrive_nm)  # stick band = backdrive / K (within 5 %)

    def test_unpowered_breakaway_equals_backdrive(self):
        s, j, sj = make("M", enable=False)
        sj.set_external_torque(lambda t, q, qd: 0.5 * t)     # 0.5 Nm per second ramp
        broke = None
        for k in range(10000):
            am.am_cycle(s)
            if abs(sj.qd) > 1e-6:
                broke = k * 1e-3 * 0.5
                break
        self.assertIsNotNone(broke)
        self.assertAlmostEqual(broke, M_BACKDRIVE, delta=0.02)

    def test_csp_tracks_and_csv_tracks(self):
        s, j, sj = make("M", mode=am.AmMode.CSP, load_inertia_kgm2=0.05)
        for _ in range(3000):
            am.am_set_position(j, 0.3)
            am.am_cycle(s)
        self.assertAlmostEqual(am.am_read_telemetry(j).q_rad, 0.3, delta=0.01)
        am.am_disable(j)
        am.am_cycle(s)
        am.am_set_mode(j, am.AmMode.CSV)
        am.am_enable(j, 1000)
        for _ in range(1500):
            am.am_set_velocity(j, 2.0)
            am.am_cycle(s)
        self.assertAlmostEqual(am.am_read_telemetry(j).qd_rad_s, 2.0, delta=0.1)

    def test_cst_torque_to_acceleration(self):
        s, j, sj = make("M", load_inertia_kgm2=0.0, friction_scale=0.0)
        for _ in range(20):
            am.am_set_torque(j, 10.0)
            am.am_cycle(s)
        t = am.am_read_telemetry(j)
        expect = 10.0 / M_JOUT * 0.020                      # below the no-load speed, so no torque roll-off
        self.assertAlmostEqual(t.qd_rad_s, expect, delta=0.1 * expect)
        self.assertAlmostEqual(t.tau_est_Nm, 10.0, delta=0.01)

    def test_current_mode_needs_explicit_enable(self):
        s, j, sj = make("M", enable=False)
        with self.assertRaises(am.AmError) as cm:
            am.am_set_mode(j, am.AmMode.CURRENT)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_NOT_SUPPORTED)
        am.am_sdo_write(j, 0x2A11, 3, b"\x01")
        am.am_set_mode(j, am.AmMode.CURRENT)
        am.am_enable(j, 1000)
        for _ in range(10):                              # short: the unloaded joint speeds up and rolls off at no-load speed
            am.am_set_current(j, 5.0)
            am.am_cycle(s)
        t = am.am_read_telemetry(j)
        self.assertAlmostEqual(t.iq_A, 5.0, delta=0.01)
        with self.assertRaises(am.AmError) as cm:
            am.am_set_mode(j, am.AmMode.CURRENT)         # entry only before Operation enabled
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STATE)


class Encoders(unittest.TestCase):
    def test_quantisation(self):
        for unit in ("M", "S-D"):
            s, j, sj = make(unit, mode=am.AmMode.CST, load_inertia_kgm2=0.02, friction_scale=0.0)
            ratio = sj.ud.ratio
            motor_lsb_out = TWO_PI / 2 ** 19 / ratio
            out_lsb = TWO_PI / 2 ** 14
            seen_counts = set()
            for k in range(1500):
                am.am_set_torque(j, 3.0 if k < 700 else -3.0)
                am.am_cycle(s)
                t = am.am_read_telemetry(j)
                self.assertIsInstance(t.q_motor_counts, int)
                self.assertAlmostEqual(t.q_rad / motor_lsb_out, round(t.q_rad / motor_lsb_out), places=6)
                self.assertAlmostEqual(t.q_out_encoder_rad / out_lsb, round(t.q_out_encoder_rad / out_lsb), places=6)
                self.assertLess(abs(t.q_rad - sj.q), motor_lsb_out)
                self.assertLess(abs(t.q_out_encoder_rad - (sj.q % TWO_PI)), out_lsb + 1e-9)
                seen_counts.add(t.q_motor_counts)
            self.assertGreater(len(seen_counts), 100)
            # motor resolution is finer than the output encoder by ratio * 32 (2^19 / 2^14)
            self.assertAlmostEqual(out_lsb / motor_lsb_out, ratio * 32, places=6)


class StateMachine(unittest.TestCase):
    def test_enable_walk_order_and_reference_gate(self):
        cfg = am.AmSessionConfig(interface="sim:M")
        s = am.am_init(cfg)
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        am.am_set_mode(j, am.AmMode.CSP)
        seen = [j.tlm.state]
        j.enable_req = True
        for _ in range(40):
            am.am_cycle(s)
            st = j.tlm.state
            if not seen or seen[-1] != st:
                seen.append(st)
        S = am.AmDriveState
        self.assertEqual(seen[-4:], [S.SWITCH_ON_DISABLED, S.READY_TO_SWITCH_ON, S.SWITCHED_ON, S.OPERATION_ENABLED])
        self.assertEqual(j.tlm.ref_state, am.AmRefState.VALID)

    def test_enable_needs_mode_and_session(self):
        cfg = am.AmSessionConfig(interface="sim:M")
        s = am.am_init(cfg)
        j = am.am_open_joint(s, 0)
        with self.assertRaises(am.AmError) as cm:
            am.am_enable(j, 100)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STATE)
        am.am_start(s)
        with self.assertRaises(am.AmError) as cm:
            am.am_enable(j, 100)                         # no mode yet
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STATE)

    def test_disable_lands_in_ready_and_torque_stops(self):
        s, j, sj = make("M", load_inertia_kgm2=HEAVY)
        am.am_set_torque(j, 20.0)
        am.am_cycle(s)
        self.assertGreater(sj.torque_applied, 19.0)
        am.am_disable(j)
        am.am_cycle(s)
        am.am_cycle(s)
        self.assertEqual(j.tlm.state, am.AmDriveState.READY_TO_SWITCH_ON)       # D1
        self.assertEqual(sj.torque_applied, 0.0)

    def test_quick_stop_ramps_to_zero_speed(self):
        s, j, sj = make("M", mode=am.AmMode.CSV, load_inertia_kgm2=0.1)
        for _ in range(800):
            am.am_set_velocity(j, 3.0)
            am.am_cycle(s)
        self.assertGreater(sj.qd, 2.0)
        am.am_quick_stop(j)
        states = set()
        for _ in range(3000):
            am.am_cycle(s)
            states.add(j.tlm.state)
            if j.tlm.state == am.AmDriveState.SWITCH_ON_DISABLED:
                break
        self.assertIn(am.AmDriveState.QUICK_STOP_ACTIVE, states)
        self.assertEqual(j.tlm.state, am.AmDriveState.SWITCH_ON_DISABLED)
        self.assertLess(abs(sj.qd), 0.5)
        for _ in range(50):
            am.am_cycle(s)
        self.assertEqual(j.tlm.state, am.AmDriveState.SWITCH_ON_DISABLED)       # no automatic restart

    def test_mode_rules(self):
        s, j, sj = make("M", mode=am.AmMode.CST)
        with self.assertRaises(am.AmError) as cm:
            am.am_set_position(j, 0.1)                   # mode is CST
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STATE)
        am.am_set_mode(j, am.AmMode.IMPEDANCE)           # bumpless change while enabled
        am.am_set_impedance(j, am.AmImpedance(kp_Nm_per_rad=10.0, kd_Nm_s_per_rad=1.0))
        am.am_cycle(s)
        self.assertEqual(j.tlm.mode, am.AmMode.IMPEDANCE)
        self.assertEqual(j.tlm.state, am.AmDriveState.OPERATION_ENABLED)
        with self.assertRaises(am.AmError):
            am.am_set_mode(j, 77)
        with self.assertRaises(am.AmError):
            am.am_set_mode(j, am.AmMode.NONE)

    def test_no_torque_until_first_command_after_enable(self):
        s, j, sj = make("M", mode=am.AmMode.CSP, load_inertia_kgm2=0.05, q0_rad=1.0)
        for _ in range(20):
            am.am_cycle(s)
        self.assertLess(abs(sj.torque_applied), 1e-6)    # holds position, no jump to 0 rad

    def test_injected_fault_latches_and_needs_clear_then_enable(self):
        events = []
        cfg = am.AmSessionConfig(interface="sim:M", event_cb=events.append)
        s = am.am_init(cfg)
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        am.am_set_mode(j, am.AmMode.CST)
        am.am_enable(j, 1000)
        j.jb.inject_fault(am.FAULT_HOST_STALE)
        for _ in range(5):
            am.am_cycle(s)
        self.assertEqual(j.tlm.state, am.AmDriveState.FAULT)
        self.assertEqual(j.tlm.error_code, am.FAULT_HOST_STALE)
        self.assertEqual(am.am_fault_name(j.tlm.error_code), "HOST_STALE")
        self.assertTrue(j.tlm.fault_word & (1 << 11))
        with self.assertRaises(am.AmError) as cm:
            am.am_set_torque(j, 1.0)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_FAULT)
        self.assertEqual(am.am_get_fault_history(j)[0].code, am.FAULT_HOST_STALE)
        j.jb.clear_fault_cause()
        am.am_clear_faults(j, 500)
        self.assertEqual(j.tlm.state, am.AmDriveState.SWITCH_ON_DISABLED)
        self.assertEqual(j.tlm.ref_state, am.AmRefState.INVALID)
        am.am_enable(j, 1000)                            # reference re-qualifies, then enable works
        self.assertEqual(j.tlm.state, am.AmDriveState.OPERATION_ENABLED)
        kinds = [e.kind for e in events]
        self.assertIn(am.AmEventKind.FAULT, kinds)
        self.assertIn(am.AmEventKind.FAULT_CLEARED, kinds)

    def test_comms_loss_disarms(self):
        s, j, sj = make("M", load_inertia_kgm2=HEAVY)
        am.am_set_torque(j, 10.0)
        am.am_cycle(s)
        am.am_stop(s)
        self.assertEqual(sj.state, am.AmDriveState.SWITCH_ON_DISABLED)
        self.assertEqual(sj.tau_applied, 0.0)


class STO(unittest.TestCase):
    def test_sto_stops_torque_and_blocks_enable(self):
        events = []
        cfg = am.AmSessionConfig(interface="sim:M", event_cb=events.append)
        s = am.am_init(cfg)
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        am.am_set_mode(j, am.AmMode.CST)
        am.am_enable(j, 1000)
        sj = s.backend.joints[0]
        sj.set_load_inertia(HEAVY)
        for _ in range(10):
            am.am_set_torque(j, 20.0)
            am.am_cycle(s)
        self.assertAlmostEqual(sj.torque_applied, 20.0, places=6)
        sj.set_sto(True)
        am.am_set_torque(j, 20.0)
        am.am_cycle(s)                                   # reaction within one cycle (<= 2 ms spec)
        t = am.am_read_telemetry(j)
        self.assertEqual(sj.torque_applied, 0.0)
        self.assertTrue(t.sto_active)
        self.assertTrue(t.statusword & 0x8000)
        self.assertEqual(t.state, am.AmDriveState.SWITCH_ON_DISABLED)
        self.assertEqual(t.safety_io & (am.AM_SIO_STO1 | am.AM_SIO_STO2), 0)
        for _ in range(20):
            am.am_set_torque(j, 20.0)
            am.am_cycle(s)
            self.assertEqual(sj.torque_applied, 0.0)
        with self.assertRaises(am.AmError) as cm:
            am.am_enable(j, 100)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STO)
        sj.set_sto(False)
        for _ in range(50):
            am.am_set_torque(j, 20.0)
            am.am_cycle(s)
        self.assertEqual(j.tlm.state, am.AmDriveState.SWITCH_ON_DISABLED)       # never restarts alone
        self.assertEqual(sj.torque_applied, 0.0)
        am.am_enable(j, 1000)                            # explicit walk, reference re-established
        am.am_set_torque(j, 20.0)
        am.am_cycle(s)
        self.assertAlmostEqual(sj.torque_applied, 20.0, places=6)
        self.assertIn(am.AmEventKind.STO_CHANGED, [e.kind for e in events])
        self.assertIn("STO", am.am_status_str(am.AmStatus.AM_ERR_STO))


class Identity(unittest.TestCase):
    def test_identity_and_transparency_all_units(self):
        cfg = am.AmSessionConfig(interface="sim:S-S,S-D,M,H-F,H-T,H-X")
        s = am.am_init(cfg)
        devs = am.am_scan(s)
        self.assertEqual([d.product_code for d in devs], [2, 1, 3, 4, 5, 6])
        for d in devs:
            j = am.am_open_joint(s, d.address)
            ident = am.am_get_identity(j)
            tr = am.am_get_transparency(j)
            self.assertEqual(tr.basis, am.AmDataBasis.DESIGN_TARGET)
            self.assertGreater(tr.peak_torque_2s_Nm, tr.rated_torque_Nm)
            self.assertAlmostEqual(ident.ratio_num / ident.ratio_den, j.jb.ud.ratio, places=3)
            self.assertGreater(tr.backdrive_torque_Nm, 0.3)
            if ident.sku == am.AmSku.S_D:
                self.assertAlmostEqual(tr.backdrive_torque_Nm, 0.36, places=3)
            if ident.sku == am.AmSku.S_S:
                self.assertAlmostEqual(tr.backdrive_torque_Nm, 0.65, places=3)
        m = am.am_open_joint(s, 2)
        self.assertEqual(am.am_get_transparency(m).peak_torque_2s_Nm, 80.0)
        self.assertAlmostEqual(am.am_get_transparency(m).reflected_inertia_kgm2, M_JOUT)
        self.assertEqual(int.from_bytes(am.am_sdo_read(m, 0x2D00, 2), "little"), 80000)
        self.assertEqual(int.from_bytes(am.am_sdo_read(m, 0x2D00, 12), "little"), 1)
        with self.assertRaises(am.AmError) as cm:
            am.am_sdo_write(m, 0x6064, 0, b"\0\0\0\0")
        self.assertEqual(am.am_last_abort_code(m), 0x06010002)
        with self.assertRaises(am.AmError):
            am.am_sdo_read(m, 0x9999, 0)

    def test_user_label_and_calibration(self):
        s, j, sj = make("M", enable=False)
        am.am_set_user_label(j, "left knee")
        self.assertEqual(am.am_get_identity(j).user_label, "left knee")
        self.assertIn('"data_basis": "design_target"', am.am_read_calibration(j))


class Backends(unittest.TestCase):
    def test_unknown_interface(self):
        with self.assertRaises(am.AmError) as cm:
            am.am_init(am.AmSessionConfig(interface="hardware"))
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_PARAM)
        with self.assertRaises(am.AmError) as cm:
            am.am_init(am.AmSessionConfig(bus=am.AmBus.ETHERCAT, interface="can:virtual:x"))
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_PARAM)

    def test_control_authority(self):
        s, j, sj = make("M", mode=am.AmMode.CST, enable=False)
        self.assertEqual(am.am_get_control_bus(j), (am.AmBus.ETHERCAT, am.AmBus.ETHERCAT))
        am.am_set_control_bus(j, am.AmBus.CANFD)
        self.assertEqual(am.am_get_control_bus(j), (am.AmBus.CANFD, am.AmBus.ETHERCAT))   # effective after reset
        sj.power_cycle()
        with self.assertRaises(am.AmError) as cm:
            am.am_set_torque(j, 1.0)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_AUTHORITY)

    def test_telemetry_fields(self):
        s, j, sj = make("M", mode=am.AmMode.CST, load_inertia_kgm2=0.05)
        iq = []
        for _ in range(500):
            am.am_set_torque(j, 30.0)
            am.am_cycle(s)
            iq.append(am.am_read_telemetry(j).iq_A)
        t = am.am_read_telemetry(j)
        self.assertEqual(t.t_ns, s.t_ns)
        self.assertTrue(0 < t.vbus_V < 48.5)
        self.assertGreater(t.t_winding_C, 25.0)
        self.assertGreater(max(iq[-20:]), 0)                   # at the no-load speed the current pulses; any of the last cycles
        self.assertGreater(t.qd_motor_rad_s, t.qd_rad_s * 20)
        self.assertEqual(t.sync_flags & am.AM_SYNC_DC_LOCKED, am.AM_SYNC_DC_LOCKED)
        self.assertEqual(t.ref_state, am.AmRefState.VALID)


@unittest.skipUnless(__import__("importlib").util.find_spec("mujoco"), "mujoco not importable")
class Mujoco(unittest.TestCase):
    XML = """<mujoco><option timestep="0.0005" gravity="0 0 -9.81"/>
    <worldbody><body pos="0 0 1"><joint name="hinge" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.02" mass="3"/></body></worldbody></mujoco>"""

    def test_impedance_hold_against_gravity(self):
        import mujoco
        from am_sdk.mujoco_coupling import MujocoCoupling
        model = mujoco.MjModel.from_xml_string(self.XML)
        data = mujoco.MjData(model)
        s = am.am_init(am.AmSessionConfig(interface="sim:M"))
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        am.am_set_mode(j, am.AmMode.IMPEDANCE)
        cpl = MujocoCoupling(s, model, data, {j: "hinge"})
        am.am_enable(j, 1000)
        target = 0.6
        for _ in range(2500):
            am.am_set_impedance(j, am.AmImpedance(q_ref_rad=target, kp_Nm_per_rad=200.0, kd_Nm_s_per_rad=8.0))
            cpl.step()
        t = am.am_read_telemetry(j)
        self.assertAlmostEqual(float(data.qpos[0]), t.q_rad, delta=1e-5)
        grav = 3 * 9.81 * 0.2 * math.sin(target)
        self.assertAlmostEqual(200.0 * (target - t.q_rad), grav, delta=1.5)    # K * error balances gravity
        self.assertLessEqual(abs(j.jb.torque_applied), 80.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
