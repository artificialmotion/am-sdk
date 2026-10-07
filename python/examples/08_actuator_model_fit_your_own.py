"""Fit your own actuator: build a spec from minimal datasheet data, see what was assumed, save it.

    python 08_actuator_model_fit_your_own.py [out.json]

Replace every assumption you can with your own data (thermal time constants from a heat run, friction
from a slow-speed sweep, the torque-speed curve from a dyno) by editing the JSON; the model reads it
back with load_spec("out.json"). SIMULATED.
"""
import sys

import _path  # noqa: F401
from am_sdk.actuator_model import ActuatorModel, load_spec, spec_from_datasheet

spec = spec_from_datasheet(
    id="my-knee-actuator", vendor="me", model="prototype 1",
    rated_nm=30.0, peak_nm=90.0, mass_kg=1.1, ratio=12.0,
    rated_speed_rpm=120.0, no_load_rpm=180.0, bus_v=48.0,
    backdrive_nm=0.9,                          # measured, so not an assumption
    ambient_c=25.0, cooling="plate", mount_k_per_w=0.1, conditions_text="25 C, on an aluminium plate",
    thermal_tau_s=420.0, winding_tau_s=45.0,   # from a heat run
)
print("assumed values (replace with your data where you can):")
for a in spec.assumptions:
    print("  %-40s %-10s %s" % (a["field"], str(a["value"])[:10], a["basis"]))
act = ActuatorModel(spec)
print("continuous torque reproduced at the rated point: %.2f N m" % act.continuous_torque(spec.continuous_speed))
out = sys.argv[1] if len(sys.argv) > 1 else None
if out:
    spec.save(out)
    print("saved", out, "->", load_spec(out))
