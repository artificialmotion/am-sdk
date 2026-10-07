"""Optional: drive a MuJoCo joint through the same API calls (needs mujoco)."""
import math

import _path  # noqa: F401
import am_sdk as am

try:
    import mujoco
except ImportError:
    raise SystemExit("mujoco is not installed; the sim backend and the other examples run without it")
from am_sdk.mujoco_coupling import MujocoCoupling

XML = """<mujoco><option timestep="0.0005" gravity="0 0 -9.81"/>
<worldbody><body pos="0 0 1"><joint name="shoulder" type="hinge" axis="0 1 0"/>
<geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.025" mass="4"/></body></worldbody></mujoco>"""
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

s = am.am_init(am.AmSessionConfig(interface="sim:M"))
j = am.am_open_joint(s, 0)
am.am_start(s)
am.am_set_mode(j, am.AmMode.IMPEDANCE)
cpl = MujocoCoupling(s, model, data, {j: "shoulder"})
am.am_enable(j, 1000)
for k in range(4000):
    target = 0.8 * math.sin(2 * math.pi * 0.5 * k * 1e-3)
    rate = 0.8 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * k * 1e-3)
    am.am_set_impedance(j, am.AmImpedance(q_ref_rad=target, qd_ref_rad_s=rate, kp_Nm_per_rad=250.0,
                                          kd_Nm_s_per_rad=10.0, tau_ff_Nm=4 * 9.81 * 0.25 * math.sin(target)))
    cpl.step()
t = am.am_read_telemetry(j)
print("MuJoCo hinge driven by an M joint: q=%.3f rad (target %.3f), torque %.1f N*m, state %s"
      % (float(data.qpos[0]), target, t.tau_est_Nm, t.state.name))
am.am_deinit(s)
