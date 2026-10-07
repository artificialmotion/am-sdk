"""Actuator model basics: load a spec, read limits, run a duty cycle, read temperatures and events.

    python 05_actuator_model_basics.py [spec id or path]     (default AM-M; try RobStride-04)

SIMULATED. Lumped thermal model calibrated to the datasheet rating; see docs/ACTUATOR_MODEL.md.
"""
import math
import sys

import _path  # noqa: F401
from am_sdk.actuator_model import ActuatorModel, list_specs, load_spec

sid = sys.argv[1] if len(sys.argv) > 1 else "AM-M"
print("shipped specs:", ", ".join(list_specs()))
spec = load_spec(sid)
print("\n%s | %s" % (spec.id, spec.status))
print("rating conditions:", spec.conditions.get("text") or "-", "| stated:", spec.conditions["stated"])
print("%d assumed values:" % len(spec.assumptions))
for a in spec.assumptions:
    print("  %-40s %-12s %s" % (a["field"], str(a["value"])[:12], a["basis"][:90]))

act = ActuatorModel(spec, dt=0.001)
print("\nspeed rad/s   peak N m   continuous N m")
w_end = spec.curve("peak")[0].w_max
for f in (0.0, 0.25, 0.5, 0.75, 0.95):
    w = f * min(w_end, 50.0)
    print("  %8.2f   %8.2f   %8.2f" % (w, act.curve_limit(w, "peak"), act.curve_limit(w, "continuous")))
print("model continuous torque at the rated speed: %.2f N m (rating %.2f)"
      % (act.continuous_torque(spec.continuous_speed), spec.continuous_nm))
print("same at 40 C ambient: %.2f N m" % act.continuous_torque(spec.continuous_speed, ambient_c=40.0))

# duty cycle: 1 s at 1.6x rated torque, 1 s at 0.3x, slow oscillation, for 10 minutes at 1 kHz
print()
w0 = 0.5 * spec.continuous_speed if spec.continuous_speed else 1.0
for k in range(600_000):
    t = k * 1e-3
    tau = (1.6 if (t % 2.0) < 1.0 else 0.3) * spec.continuous_nm
    act.step(tau, w0 * math.sin(2 * math.pi * 0.25 * t))
    if k % 60_000 == 0:
        s = act.state
        print("t=%5.0f s  winding %6.1f C  housing %6.1f C  derate %.3f  peak budget %.2f s  rms10s %6.2f N m"
              % (t, s.winding_c, s.housing_c, s.derate, s.peak_budget_s, act.duty.rms(10.0)))
print("\nevent counts:", act.event_counts)
print("duty summary:", {k: round(v, 3) if isinstance(v, float) else v for k, v in act.duty.summary().items()})
