"""Apply actuator models to ANY MJCF: give the model file and a joint -> spec mapping.

    python 07_actuator_model_any_mjcf.py robot.xml mapping.json [seconds] [--keep-plant]

mapping.json: {"left_knee": "AM-M", "left_hip_pitch": "AM-H-T", "right_knee": "path/to/my_spec.json", ...}
Joints must be hinge or slide. If a joint has a motor actuator in the MJCF, the bank writes data.ctrl
(gear and gain respected) and widens its ctrlrange/forcerange to the spec's peak if needed; otherwise it
writes data.qfrc_applied. Armature, frictionloss and damping are copied from the spec (--keep-plant
keeps the MJCF's values). The demo holds every mapped joint at its initial position with a PD law,
so you see torque limits, heating and events on your own robot.

Without arguments it writes and uses a two-joint arm. Needs mujoco. SIMULATED.
"""
import json
import os
import sys
import tempfile

import _path  # noqa: F401

try:
    import mujoco
except ImportError:
    raise SystemExit("mujoco is not installed: pip install mujoco")
from am_sdk.actuator_model.mujoco_bridge import MujocoActuatorBank

ARM = """<mujoco><option timestep="0.001"/><worldbody><body pos="0 0 1.5">
<joint name="shoulder" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.04" mass="6"/>
<body pos="0.5 0 0"><joint name="elbow" type="hinge" axis="0 1 0"/>
<geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" mass="3"/></body></body></worldbody></mujoco>"""

args = [a for a in sys.argv[1:] if not a.startswith("--")]
keep = "--keep-plant" in sys.argv
if len(args) >= 2:
    xml_path, map_path = args[0], args[1]
    with open(map_path, "r", encoding="utf-8") as f:
        mapping = json.load(f)
else:
    xml_path = os.path.join(tempfile.mkdtemp(), "arm.xml")
    with open(xml_path, "w") as f:
        f.write(ARM)
    mapping = {"shoulder": "AM-H-T", "elbow": "AM-M"}
seconds = float(args[2]) if len(args) > 2 else 20.0

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
bank = MujocoActuatorBank(model, data, mapping, apply_plant_params=not keep)
adr = {}
for j in mapping:
    i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    adr[j] = (model.jnt_qposadr[i], model.jnt_dofadr[i])
q0 = {j: float(data.qpos[qa]) for j, (qa, dv) in adr.items()}
for k in range(int(seconds / model.opt.timestep)):
    cmd = {j: 400.0 * (q0[j] - data.qpos[qa]) - 20.0 * data.qvel[dv] for j, (qa, dv) in adr.items()}
    bank.step(cmd)
for j, s in bank.summary().items():
    print("%-20s %-28s winding %6.1f C  rms(10 s) %7.2f N m  saturated %5.2f s  events %s"
          % (j, s["id"], s["winding_c"], s["rms_10s_nm"], s["t_saturated_s"], s["events"]))
