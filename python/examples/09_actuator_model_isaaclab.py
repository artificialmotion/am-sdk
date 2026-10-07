# SPDX-License-Identifier: Apache-2.0
"""Isaac Lab: an ActuatorBase-compatible actuator with thermal-aware limits for any spec.

UNTESTED ON ISAAC LAB: written against the documented Isaac Lab 2.x actuator API. The numeric core
(BatchActuatorModel) is unit-tested with numpy and torch. Run this inside an Isaac Lab environment.
"""
import _path  # noqa: F401
from am_sdk.actuator_model.isaac_lab import make_actuator_cfg

try:
    knees = make_actuator_cfg("AM-M", joint_names_expr=[".*_knee"], stiffness=150.0, damping=6.0, physics_dt=0.005)
    hips = make_actuator_cfg({".*_hip_pitch": "AM-H-T", ".*_hip_roll": "AM-H-F"}, joint_names_expr=[".*_hip_.*"],
                             stiffness=200.0, damping=8.0, physics_dt=0.005)
except ImportError as e:
    raise SystemExit(str(e))
print(knees)
print(hips)
# In your ArticulationCfg: actuators={"knees": knees, "hips": hips}
# At run time: env.scene["robot"].actuators["knees"].model.tw  -> winding temperature per env and joint (C)
