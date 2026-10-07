# SPDX-License-Identifier: Apache-2.0
"""Tests for am_sdk.actuator_model (spec, runtime model, batch, MuJoCo bank, Isaac Lab glue on a stub)."""
import dataclasses
import json
import math
import os
import sys
import time
import types
import unittest
import warnings

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import am_sdk.actuator_model as AM  # noqa: E402
from am_sdk import units as U  # noqa: E402
from am_sdk.actuator_model import (ActuatorModel, BatchActuatorModel, SpecError, list_specs, load_spec,  # noqa: E402
                                   spec_from_datasheet, validate_spec)

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None
try:
    import torch
except ImportError:  # pragma: no cover
    torch = None
try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None

RPM = 2 * math.pi / 60
AM_IDS = ["AM-" + u for u in U.UNIT_IDS]
EX_IDS = ["example_qdd_9to1", "example_planetary_36to1", "example_harmonic_100to1"]


class Specs(unittest.TestCase):
    def test_all_shipped_specs_validate(self):
        reg = list_specs()
        for i in AM_IDS + EX_IDS:
            self.assertIn(i, reg)
            with open(reg[i], "r", encoding="utf-8") as f:
                self.assertEqual(validate_spec(json.load(f)), [], i)

    def test_am_specs_match_sdk_unit_data(self):
        for u in U.UNIT_IDS:
            ud = U.load_unit(u)
            sp = load_spec("AM-" + u)
            self.assertEqual(sp.peak_nm, ud.peak_nm)
            self.assertEqual(sp.continuous_nm, ud.rated_nm)
            self.assertEqual(sp.inertia_kgm2, ud.armature_kgm2)
            self.assertEqual(sp.ratio, ud.ratio)
            self.assertEqual(sp.backdrive_nm, ud.backdrive_nm)
            self.assertEqual(sp.thermal["nodes"]["tau_s"], ud.thermal_tau_s)
            m = ActuatorModel(sp)
            m36 = ActuatorModel(sp, bus_v=36)
            for w in [0.0, 1.0, 3.0, 5.0, 7.0, 9.0, 11.0]:
                for kind in ("peak", "continuous"):
                    ref = ud.model.torque_limit(w, "48", kind)
                    got = m.curve_limit(w, kind)
                    self.assertAlmostEqual(got, 0.0 if ref is None else ref, delta=1e-6, msg="%s %s %g" % (u, kind, w))
                    ref = ud.model.torque_limit(w, "36", kind)
                    self.assertAlmostEqual(m36.curve_limit(w, kind), 0.0 if ref is None else ref, delta=1e-6)
            if not ud.thermal_tau_published:
                self.assertTrue(sp.has_assumption("thermal.nodes.tau_s"))

    def test_example_specs_are_unnamed_illustrative_and_flag_conditions(self):
        face = {"example_qdd_9to1": (20, 60, 1.0, 100), "example_planetary_36to1": (40, 120, 1.5, 30),
                "example_harmonic_100to1": (80, 240, 2.5, 20)}
        for i, (rated, peak, mass, rpm) in face.items():
            sp = load_spec(i)
            self.assertEqual(sp.continuous_nm, rated)
            self.assertEqual(sp.peak_nm, peak)
            self.assertEqual(sp.mass_kg, mass)
            if rpm is not None:
                self.assertAlmostEqual(sp.continuous_speed, rpm * RPM, places=9)
            self.assertFalse(sp.conditions["stated"], i)
            self.assertEqual(sp.raw["vendor"], "", i)
            self.assertIn("not any real product", sp.raw["status"], i)
            for f in ("mechanical.reflected_inertia_kgm2", "thermal.nodes.c_total_j_per_k", "ratings.peak_duration_s",
                      "ratings.conditions.ambient_c"):
                self.assertTrue(sp.has_assumption(f), "%s %s" % (i, f))

    def test_validation_catches_errors(self):
        d = load_spec("AM-M").to_dict()
        d["ratings"]["continuous_nm"] = 500
        d["thermal"]["trip_c"] = 50
        d["torque_speed"][0]["points"] = [[10, 1], [5, 1]]
        errs = validate_spec(d)
        self.assertTrue(any("exceeds" in e for e in errs))
        self.assertTrue(any("trip_c" in e for e in errs))
        self.assertTrue(any("non-decreasing" in e for e in errs))
        with self.assertRaises(SpecError):
            load_spec(d)
        with self.assertRaises(SpecError):
            load_spec("no-such-unit")

    def test_builder_minimal_and_roundtrip(self):
        sp = spec_from_datasheet(id="x", rated_nm=20, peak_nm=50, mass_kg=0.8, ratio=10, rated_speed_rpm=150)
        fields = {a["field"] for a in sp.assumptions}
        for f in ("mechanical.reflected_inertia_kgm2", "mechanical.gear_efficiency", "friction.coulomb_nm",
                  "ratings.no_load_speed_rad_s", "thermal.nodes.tau_housing_s", "thermal.nodes.tau_winding_s",
                  "thermal.derate_c", "thermal.trip_c"):
            self.assertIn(f, fields)
        m = ActuatorModel(sp)
        self.assertAlmostEqual(m.curve_limit(0.0), 50.0)
        self.assertAlmostEqual(m.curve_limit(150 * RPM, "continuous"), 20.0, places=6)
        self.assertAlmostEqual(m.curve_limit(1.25 * 150 * RPM), 0.0, places=6)
        self.assertEqual(m.curve_limit(2.0 * 150 * RPM), 0.0)
        sp2 = load_spec(sp.to_dict())
        self.assertEqual(sp2.to_dict(), sp.to_dict())
        # electrical calibration when Kt, R and ratio are given
        sp3 = spec_from_datasheet(id="y", rated_nm=20, peak_nm=50, mass_kg=0.8, ratio=10, rated_speed_rpm=150,
                                  kt_nm_per_a=0.1, phase_resistance_ohm=0.2, current_limit_a=40)
        m3 = ActuatorModel(sp3)
        self.assertEqual(m3.calibration, "electrical")
        self.assertAlmostEqual(m3.torque_limits(0.0)[0], min(50.0, 40 * 0.1 * 10 * 0.9), places=6)
        p_cu, _ = m3.losses(20.0, 0.0, winding_c=25.0)
        i = 20.0 / 0.9 / 10 / 0.1
        self.assertAlmostEqual(p_cu, 1.5 * 0.2 * i * i, places=6)


class Model(unittest.TestCase):
    def test_rating_reproduced_at_rated_conditions(self):
        for i in AM_IDS + EX_IDS:
            m = ActuatorModel(i)
            sp = m.spec
            self.assertAlmostEqual(m.continuous_torque(sp.continuous_speed), sp.continuous_nm,
                                   delta=1e-3 * sp.continuous_nm, msg=i)
            self.assertAlmostEqual(m.steady_state_winding_c(sp.continuous_nm, sp.continuous_speed),
                                   sp.thermal["rated_winding_c"], delta=0.01)

    def test_hotter_ambient_lowers_continuous_torque(self):
        m = ActuatorModel("AM-M")
        w = m.spec.continuous_speed
        self.assertLess(m.continuous_torque(w, ambient_c=40.0), m.continuous_torque(w, ambient_c=25.0))
        better = ActuatorModel("AM-M", cooling_scale=2.0)
        self.assertGreater(better.continuous_torque(w), m.continuous_torque(w))

    def test_torque_never_above_curve_and_braking_capped(self):
        m = ActuatorModel("AM-M", peak_policy="thermal")
        for w in [-14, -10, -5, -1, 0, 1, 5, 10, 12.6, 14]:
            pos, neg = m.torque_limits(w)
            pk = m.curve_limit(w)
            if w > 0:
                self.assertAlmostEqual(pos, pk)
                self.assertAlmostEqual(neg, 80.0)
            elif w < 0:
                self.assertAlmostEqual(neg, pk)
                self.assertAlmostEqual(pos, 80.0)
            tau = m.step(1e4, w)
            self.assertLessEqual(tau, max(pos, 0.0) + 1e-9)
        self.assertEqual(m.torque_limits(20.0)[0], 0.0)

    def test_peak_budget(self):
        m = ActuatorModel("AM-M", dt=0.001)
        n_peak = 0
        for k in range(4000):
            tau = m.step(80.0, 0.0)
            if tau > 79.9:
                n_peak += 1
        self.assertEqual(n_peak, 2000)
        self.assertEqual(m.event_counts.get(AM.PEAK_BUDGET_EMPTY), 1)
        self.assertAlmostEqual(m.state.tau_nm, m.curve_limit(0.0, "continuous") * m.state.derate, places=9)
        # recovers: 0.5 s rearm at 0.1 s/s -> 5 s below continuous
        for k in range(5100):
            m.step(0.0, 0.0)
        self.assertTrue(m.state.peak_available)
        self.assertEqual(m.event_counts.get(AM.PEAK_REARMED), 1)

    def test_derate_holds_winding_at_threshold(self):
        m = ActuatorModel("AM-M", dt=0.05, peak_policy="thermal")
        for k in range(int(4000 / 0.05)):
            m.step(80.0, 0.0)
        s = m.state
        self.assertGreater(m.event_counts.get(AM.DERATE_START, 0), 0)
        self.assertLess(s.derate, 1.0)
        self.assertAlmostEqual(s.winding_c, m.derate_c, delta=1.0)
        self.assertFalse(s.tripped)
        self.assertLess(s.tau_nm, 45.0)

    def test_trip_and_clear(self):
        m = ActuatorModel("AM-M", initial_c=135.0, dt=0.01)
        tau = m.step(50.0, 0.0)
        self.assertEqual(tau, 0.0)
        self.assertTrue(m.state.tripped)
        self.assertEqual(m.event_counts[AM.TRIP], 1)
        for k in range(20000):
            m.step(0.0, 0.0)
            if not m.state.tripped:
                break
        self.assertFalse(m.state.tripped)
        self.assertLessEqual(m.state.winding_c, m.trip_c - m.trip_hyst + 1e-9)
        self.assertEqual(m.event_counts[AM.TRIP_CLEAR], 1)

    def test_saturation_events_and_duty(self):
        seen = []
        m = ActuatorModel("AM-H-T", on_event=seen.append)
        for k in range(100):
            m.step(500.0, 0.0)
        for k in range(100):
            m.step(10.0, 0.0)
        kinds = [e.kind for e in seen]
        self.assertEqual(kinds.count(AM.SATURATION_START), 1)
        self.assertEqual(kinds.count(AM.SATURATION_END), 1)
        self.assertAlmostEqual(m.duty.t_saturated, 0.1, places=6)
        self.assertAlmostEqual(m.duty.t, 0.2, places=6)
        m2 = ActuatorModel("AM-M")
        for k in range(20000):
            m2.step(10.0, 1.0)
        self.assertAlmostEqual(m2.duty.rms(1.0), 10.0, places=3)
        self.assertAlmostEqual(m2.duty.energy_out_j, 10.0 * 1.0 * 20.0, places=6)
        self.assertGreater(m2.duty.energy_loss_j, 0.0)

    def test_matches_sdk_first_order_thermal(self):
        """With mechanical heating off and no copper tempco, the 1-node model is the SDK simulator's
        normalised model: T_w = T_a + 80 K * x, x' = ((tau/rated)^2 - x)/tau."""
        d = load_spec("AM-M").to_dict()
        d["thermal"]["mechanical_losses_heat_actuator"] = False
        d["thermal"]["copper_tempco_per_k"] = 0.0
        m = ActuatorModel(d, dt=0.01, peak_policy="thermal")
        ud = U.load_unit("M")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            ts = ud.model.ThermalState(tau_s=ud.thermal_tau_s, cont_torque=ud.rated_nm)
        for k in range(30000):
            m.step(30.0, 0.0)
            ts.step(30.0, 0.01)
        self.assertAlmostEqual(m.state.winding_c, 25.0 + U.WINDING_RISE_AT_RATED_K * ts.x, delta=0.05)

    def test_two_node_winding_leads_housing(self):
        sp = spec_from_datasheet(id="z", rated_nm=20, peak_nm=60, mass_kg=1.0, rated_speed_rpm=100)
        m = ActuatorModel(sp, dt=0.01, peak_policy="thermal")
        for k in range(1000):
            m.step(40.0, 0.0)
        self.assertGreater(m.state.winding_c - 25, 2.0 * (m.state.housing_c - 25))
        m2 = ActuatorModel(sp, dt=1.0)
        for k in range(20000):
            m2.step(sp.continuous_nm, sp.continuous_speed)
        self.assertAlmostEqual(m2.state.winding_c, 105.0, delta=0.05)

    def test_speed_scaled_bus(self):
        m48 = ActuatorModel("AM-M", bus_v=48)
        m60 = ActuatorModel("AM-M", bus_v=60)
        w = 10.0
        self.assertAlmostEqual(m60.curve_limit(w), m48.curve_limit(w * 48.0 / 60.0), places=9)

    def test_friction(self):
        sp = load_spec("AM-M").to_dict()
        sp["friction"]["stribeck"] = {"static_nm": 2.0, "velocity_rad_s": 0.05}
        sp["friction"]["viscous_nm_s_per_rad"] = 0.1
        m = ActuatorModel(sp)
        self.assertAlmostEqual(m.friction(1.0), 1.4 + 0.1, places=6)
        self.assertGreater(m.friction(0.02), 1.4)
        self.assertAlmostEqual(m.friction(-1.0), -1.5, places=6)

    def test_1khz_capable(self):
        m = ActuatorModel("AM-H-X")
        t0 = time.perf_counter()
        for k in range(10000):
            m.step(100.0 * math.sin(k * 1e-3), 3.0 * math.cos(k * 1e-3))
        el = time.perf_counter() - t0
        self.assertLess(el, 2.0, "10k steps took %.3f s" % el)


@unittest.skipIf(np is None, "numpy not installed")
class Batch(unittest.TestCase):
    def _compare(self, backend):
        ids = ["AM-M", "AM-H-T", "example_qdd_9to1", "example_harmonic_100to1"]
        N, dt = 3, 0.002
        bank = BatchActuatorModel(ids, num_envs=N, dt=dt, backend=backend,
                                  dtype=(torch.float64 if backend == "torch" else None))
        refs = [[ActuatorModel(i, dt=dt) for i in ids] for _ in range(N)]
        rng = np.random.default_rng(1)
        for k in range(3000):
            tc = rng.normal(0, 1, (N, len(ids))) * np.array([60, 150, 90, 30])
            om = rng.normal(0, 1, (N, len(ids))) * np.array([5, 5, 10, 3])
            if backend == "torch":
                out = bank.step(torch.as_tensor(tc, dtype=torch.float64), torch.as_tensor(om, dtype=torch.float64))
                out = out.cpu().numpy()
                tw = bank.tw.cpu().numpy()
            else:
                out = bank.step(tc, om)
                tw = bank.tw
            for e in range(N):
                for j, i in enumerate(ids):
                    r = refs[e][j]
                    ref = r.step(float(tc[e, j]), float(om[e, j]))
                    self.assertAlmostEqual(float(out[e, j]), ref, delta=0.01 * r.spec.peak_nm, msg="%s k=%d" % (i, k))
                    self.assertAlmostEqual(float(tw[e, j]), r.state.winding_c, delta=0.05)

    def test_numpy_matches_scalar(self):
        self._compare("numpy")

    @unittest.skipIf(torch is None, "torch not installed")
    def test_torch_matches_scalar(self):
        self._compare("torch")

    def test_reset_subset(self):
        bank = BatchActuatorModel(["AM-M"], num_envs=4, dt=0.01, backend="numpy")
        for k in range(1000):
            bank.step(np.full((4, 1), 60.0), np.zeros((4, 1)))
        hot = bank.tw.copy()
        bank.reset(np.array([1, 3]))
        self.assertTrue(np.all(bank.tw[[1, 3]] == 25.0))
        self.assertTrue(np.all(bank.tw[[0, 2]] == hot[[0, 2]]))
        self.assertGreater(bank.counts["peak_budget_empty"][0, 0], 0)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Mujoco(unittest.TestCase):
    def _leg(self, with_motors=True):
        from am_sdk.actuator_model.mujoco_bridge import mjcf_defaults_block, mjcf_snippet
        hip, knee = mjcf_snippet("AM-H-T", "hip"), mjcf_snippet("AM-M", "knee")
        acts = (hip["actuator"] + knee["actuator"]) if with_motors else ""
        xml = """<mujoco><option timestep="0.001"/>%s
<worldbody><body name="thigh" pos="0 0 1"><joint name="hip" type="hinge" axis="0 1 0" %s/>
<geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04" mass="5"/>
<body name="shank" pos="0 0 -0.4"><joint name="knee" type="hinge" axis="0 1 0" %s/>
<geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="3"/></body></body></worldbody>
<actuator>%s</actuator></mujoco>""" % (mjcf_defaults_block(["AM-H-T", "AM-M"]), hip["joint_attrs"],
                                       knee["joint_attrs"], acts)
        m = mujoco.MjModel.from_xml_string(xml)
        return m, mujoco.MjData(m)

    def test_snippet_sets_plant_params(self):
        m, d = self._leg()
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "knee")
        self.assertAlmostEqual(float(m.dof_armature[m.jnt_dofadr[jid]]), 0.03)
        self.assertAlmostEqual(float(m.dof_frictionloss[m.jnt_dofadr[jid]]), 1.4)
        self.assertEqual(m.nu, 2)

    def _run(self, with_motors):
        from am_sdk.actuator_model.mujoco_bridge import MujocoActuatorBank
        m, d = self._leg(with_motors)
        bank = MujocoActuatorBank(m, d, {"hip": "AM-H-T", "knee": "AM-M"})
        for k in range(3000):
            lim = [bank["hip"].torque_limits(float(d.qvel[0])), bank["knee"].torque_limits(float(d.qvel[1]))]
            out = bank.step({"hip": 1000.0, "knee": -1000.0})
            self.assertLessEqual(out[0], lim[0][0] + 1e-9)
            self.assertGreaterEqual(out[1], -lim[1][1] - 1e-9)
        self.assertGreater(bank["hip"].event_counts.get(AM.SATURATION_START, 0), 0)
        self.assertGreater(bank["hip"].state.winding_c, 25.0)
        return m, d, bank

    def test_bank_with_motor_actuators(self):
        m, d, bank = self._run(True)
        self.assertAlmostEqual(float(d.actuator_force[0]), bank.last_tau[0], places=6)

    def test_bank_with_qfrc_applied(self):
        m, d, bank = self._run(False)
        self.assertAlmostEqual(float(d.qfrc_applied[0]), bank.last_tau[0], places=9)

    def test_callback(self):
        from am_sdk.actuator_model.mujoco_bridge import MujocoActuatorBank
        m, d = self._leg()
        bank = MujocoActuatorBank(m, d, {"hip": "AM-H-T", "knee": "AM-M"})
        bank.install_callback(lambda mm, dd: [10.0, 5.0])
        try:
            for k in range(10):
                mujoco.mj_step(m, d)
        finally:
            bank.remove_callback()
        self.assertGreater(bank["hip"].state.t, 0.0)


@unittest.skipIf(torch is None, "torch not installed (Isaac Lab glue test needs torch)")
class IsaacLabGlueOnStub(unittest.TestCase):
    """Exercises isaac_lab.py against a minimal stub of the documented Isaac Lab API. Not Isaac Lab itself."""

    def setUp(self):
        self._saved = {k: sys.modules.get(k) for k in ("isaaclab", "isaaclab.actuators", "isaaclab.utils")}

        class IdealPDActuator:
            def __init__(self, cfg, joint_names, joint_ids, num_envs, device, **kw):
                self.cfg, self._joint_names, self._num_envs, self._device = cfg, joint_names, num_envs, device
                n = len(joint_names)
                self.stiffness = torch.full((num_envs, n), float(cfg.stiffness))
                self.damping = torch.full((num_envs, n), float(cfg.damping))

            @property
            def joint_names(self):
                return self._joint_names

            def reset(self, env_ids):
                pass

        @dataclasses.dataclass
        class IdealPDActuatorCfg:
            joint_names_expr: list = None
            effort_limit: object = None
            effort_limit_sim: object = None
            velocity_limit: object = None
            stiffness: object = None
            damping: object = None
            armature: object = None
            friction: object = None
            class_type: type = None

        pkg = types.ModuleType("isaaclab")
        act = types.ModuleType("isaaclab.actuators")
        act.IdealPDActuator, act.IdealPDActuatorCfg = IdealPDActuator, IdealPDActuatorCfg
        ut = types.ModuleType("isaaclab.utils")
        ut.configclass = dataclasses.dataclass
        sys.modules.update({"isaaclab": pkg, "isaaclab.actuators": act, "isaaclab.utils": ut})
        import am_sdk.actuator_model.isaac_lab as IL
        IL._CLASSES = None
        self.IL = IL

    def tearDown(self):
        for k, v in self._saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
        self.IL._CLASSES = None

    def test_cfg_and_compute(self):
        cfg = self.IL.make_actuator_cfg({"knee": "AM-M", "hip": "AM-H-T"}, joint_names_expr=["knee", "hip"],
                                        stiffness=100.0, damping=5.0, physics_dt=0.005)
        self.assertEqual(cfg.armature, {"knee": 0.03, "hip": 0.0707})
        self.assertEqual(cfg.effort_limit_sim, {"knee": 80, "hip": 210.0})
        a = cfg.class_type(cfg, ["knee", "hip"], [0, 1], 2, "cpu")
        ca = types.SimpleNamespace(joint_positions=torch.full((2, 2), 10.0), joint_velocities=torch.zeros(2, 2),
                                   joint_efforts=torch.zeros(2, 2))
        out = a.compute(ca, torch.zeros(2, 2), torch.zeros(2, 2))
        self.assertTrue(torch.allclose(out.joint_efforts[:, 0], torch.tensor(80.0)))
        self.assertTrue(torch.allclose(out.joint_efforts[:, 1], torch.tensor(210.0)))
        self.assertIsNone(out.joint_positions)
        a.reset([0])


if __name__ == "__main__":
    unittest.main()
