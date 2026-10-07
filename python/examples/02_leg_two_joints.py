"""Two-joint leg-like example: hip and knee of a planar two-link leg, squat motion.

Hip is an H-F, knee is an M. Gravity acts on the links through the sim backend's
external-torque hook. The controller is a joint-space impedance law with gravity
feed-forward, written only with am_* calls.
"""
import math

import _path  # noqa: F401
import am_sdk as am

G = 9.81
M1, L1 = 4.0, 0.40      # thigh
M2, L2 = 3.0, 0.40      # shank
# angles from the downward vertical; knee angle is relative to the thigh


def gravity(q1, q2):
    """Gravity torques the joints must hold (N*m)."""
    tau_knee = M2 * G * (L2 / 2) * math.sin(q1 + q2)
    tau_hip = G * (M1 * L1 / 2 + M2 * L1) * math.sin(q1) + tau_knee
    return tau_hip, tau_knee


cfg = am.AmSessionConfig(interface="sim:H-F,M", cycle_us=1000,
                         sim_options=[am.SimOptions(load_inertia_kgm2=M1 * L1 ** 2 / 3 + M2 * L1 ** 2),
                                      am.SimOptions(load_inertia_kgm2=M2 * L2 ** 2 / 3)])
s = am.am_init(cfg)
hip, knee = am.am_open_joint(s, 0), am.am_open_joint(s, 1)
sim_hip, sim_knee = s.backend.joints

# the plant sees gravity from the other joint's state
sim_hip.set_external_torque(lambda t, q, qd: -gravity(sim_hip.q, sim_knee.q)[0])
sim_knee.set_external_torque(lambda t, q, qd: -gravity(sim_hip.q, sim_knee.q)[1])

am.am_start(s)
for j in (hip, knee):
    am.am_set_mode(j, am.AmMode.IMPEDANCE)
    am.am_enable(j, 1000)

KP = {0: 600.0, 1: 400.0}
KD = {0: 25.0, 1: 15.0}
worst = {0: 0.0, 1: 0.0}
for k in range(6000):
    t = k * 1e-3
    squat = 0.5 * (1 - math.cos(2 * math.pi * 0.25 * t))          # 0..1..0 every 4 s
    q_ref = (-0.6 * squat, 1.2 * squat)                             # thigh back, knee bent
    th, tk = am.am_read_telemetry(hip), am.am_read_telemetry(knee)
    gh, gk = gravity(th.q_rad, tk.q_rad)
    for i, (j, tl) in enumerate(((hip, th), (knee, tk))):
        am.am_set_impedance(j, am.AmImpedance(q_ref_rad=q_ref[i], kp_Nm_per_rad=KP[i], kd_Nm_s_per_rad=KD[i],
                                              tau_ff_Nm=(gh, gk)[i]))
        worst[i] = max(worst[i], abs(q_ref[i] - tl.q_rad))
    am.am_cycle(s)

for name, j, i in (("hip  (H-F)", hip, 0), ("knee (M)  ", knee, 1)):
    tl = am.am_read_telemetry(j)
    print("%s max tracking error %.3f rad, torque now %.1f N*m, winding %.2f C, state %s"
          % (name, worst[i], tl.tau_est_Nm, tl.t_winding_C, tl.state.name))
am.am_deinit(s)
