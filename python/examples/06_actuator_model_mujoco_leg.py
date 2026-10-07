# SPDX-License-Identifier: Apache-2.0
"""A simple leg in MuJoCo (vertical rail, hip + knee, foot on the floor) doing squats with a 12 kg body.
Each joint runs an ActuatorModel: speed- and temperature-dependent torque limit, heating, peak budget.

    python 06_actuator_model_mujoco_leg.py [hip spec] [knee spec] [seconds]
    python 06_actuator_model_mujoco_leg.py AM-H-T AM-M 60
    python 06_actuator_model_mujoco_leg.py example_harmonic_100to1 example_qdd_9to1 60

Needs mujoco (pip install mujoco). SIMULATED.
"""
import math
import sys

import _path  # noqa: F401

try:
    import mujoco
except ImportError:
    raise SystemExit("mujoco is not installed: pip install mujoco")
from am_sdk.actuator_model.mujoco_bridge import MujocoActuatorBank, mjcf_defaults_block, mjcf_snippet

hip_id = sys.argv[1] if len(sys.argv) > 1 else "AM-H-T"
knee_id = sys.argv[2] if len(sys.argv) > 2 else "AM-M"
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0


def leg_xml(hip_spec, knee_spec, body_kg=12.0):
    hip, knee = mjcf_snippet(hip_spec, "hip"), mjcf_snippet(knee_spec, "knee")
    return """<mujoco model="am_leg">
  <option timestep="0.001"/>
  %s
  %s
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="base" pos="0 0 0.85">
      <joint name="rail" type="slide" axis="0 0 1"/>
      <geom type="box" size="0.1 0.1 0.05" mass="%g"/>
      <body name="thigh">
        <joint name="hip" type="hinge" axis="0 1 0" %s/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04" mass="4"/>
        <body name="shank" pos="0 0 -0.4">
          <joint name="knee" type="hinge" axis="0 1 0" %s/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="2"/>
          <geom name="foot" type="sphere" pos="0 0 -0.4" size="0.04" mass="0.3" friction="1.5"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    %s
    %s
  </actuator>
</mujoco>""" % (hip["comment"], mjcf_defaults_block([hip_spec, knee_spec]), body_kg, hip["joint_attrs"],
                 knee["joint_attrs"], hip["actuator"], knee["actuator"])


model = mujoco.MjModel.from_xml_string(leg_xml(hip_id, knee_id))
data = mujoco.MjData(model)
data.qpos[1], data.qpos[2] = 0.6, -1.2     # start bent
mujoco.mj_forward(model, data)
bank = MujocoActuatorBank(model, data, {"hip": hip_id, "knee": knee_id}, apply_plant_params=False)
jid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ("hip", "knee")}
qa = {n: model.jnt_qposadr[j] for n, j in jid.items()}
da = {n: model.jnt_dofadr[j] for n, j in jid.items()}
kp, kd = 600.0, 30.0
dt = model.opt.timestep
for k in range(int(seconds / dt)):
    t = k * dt
    knee_ref = -(1.2 + 0.6 * math.sin(2 * math.pi * 0.5 * t))    # squat at 0.5 Hz
    ref = {"hip": -0.5 * knee_ref, "knee": knee_ref}
    cmd = {j: kp * (ref[j] - data.qpos[qa[j]]) - kd * data.qvel[da[j]] for j in ref}
    bank.step(cmd)
    if k % int(5.0 / dt) == 0:
        line = []
        for j in ("hip", "knee"):
            s = bank[j].state
            lim = s.limit_pos_nm if s.tau_nm >= 0 else s.limit_neg_nm
            line.append("%s %6.1f N m (limit %5.1f) %5.1f C derate %.2f" % (j, s.tau_nm, lim, s.winding_c, s.derate))
        print("t=%5.1f s  base z %.3f m | %s" % (t, data.qpos[0] + 0.85, " | ".join(line)))
for j in ("hip", "knee"):
    a = bank[j]
    d = a.duty.summary()
    print("%s (%s): max winding %.1f C, rms(10 s) %.1f N m, saturated %.1f s, above continuous %.1f s, events %s"
          % (j, a.spec.id, d["max_winding_c"], d["rms_10s_nm"], d["t_saturated_s"], d["t_above_continuous_s"],
             a.event_counts))
