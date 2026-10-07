"""Clip C: one joint following a joint_trajectory_controller-style trajectory through the SDK simulator. SIMULATED.

Not an rviz or ROS recording: ROS is not involved. The trajectory is the waypoint list of
ros2/am_ros2_demo/scripts/demo_commands.py (0.8 rad at 1 s, -0.8 rad at 3 s, 0 at 4.5 s), interpolated
linearly as a position-only goal, and sent to the simulated M joint in impedance mode with the demo's
gains (K 150 N*m/rad, D 6 N*m*s/rad) and the demo arm inertia (0.05 kg m^2, no gravity: vertical axis).
Output: docs/media/ros2_trajectory.{mp4,gif}
"""
import math

from common import *  # noqa: F401,F403
import am_sdk as am

WAYPOINTS = [(0.0, 0.0), (1.0, 0.8), (3.0, -0.8), (4.5, 0.0)]   # (t s, q rad); first is the start point
K, D, J_ARM, UNIT = 150.0, 6.0, 0.05, "M"
T_END, DT = 6.5, 1e-3


def traj(t):
    for (t0, q0), (t1, q1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        if t < t1:
            v = (q1 - q0) / (t1 - t0)
            return q0 + v * (t - t0), v
    return WAYPOINTS[-1][1], 0.0


opts = am.SimOptions(load_inertia_kgm2=J_ARM, q0_rad=0.0)
s = am.am_init(am.AmSessionConfig(interface="sim:" + UNIT, cycle_us=1000, sim_options=[opts]))
j = am.am_open_joint(s, 0)
am.am_start(s)
am.am_set_mode(j, am.AmMode.IMPEDANCE)
am.am_enable(j, timeout_ms=1000)
rows = []
t0 = am.am_read_telemetry(j).t_ns * 1e-9
for k in range(int(T_END / DT)):
    qr, vr = traj(k * DT)
    am.am_set_impedance(j, am.AmImpedance(q_ref_rad=qr, qd_ref_rad_s=vr, kp_Nm_per_rad=K, kd_Nm_s_per_rad=D))
    am.am_cycle(s)
    tl = am.am_read_telemetry(j)
    rows.append((tl.t_ns * 1e-9 - t0, qr, tl.q_rad, tl.tau_est_Nm))
am.am_deinit(s)
R = np.array(rows)
t, qref, q, tau = R.T
err = np.abs(q - qref)
print("max |error| %.3f rad, final error %.4f rad, peak est torque %.1f N*m" % (err.max(), err[-1], np.abs(tau).max()))

N_FR, FPS = int(T_END * 25), 25
fig = new_fig()
title(fig, "Joint trajectory through the SDK simulator",
      "%s joint, impedance K %.0f N*m/rad, D %.0f N*m*s/rad, arm %.2f kg*m^2. Waypoints as in the ROS 2 demo." % (UNIT, K, D, J_ARM))
caption(fig, "Simulated. SDK simulator, same plugin core as the ROS 2 driver. Not an rviz recording.")
ax0 = fig.add_axes([0.02, 0.10, 0.30, 0.72])
ax0.set_xlim(-0.5, 0.5); ax0.set_ylim(-0.5, 0.5); ax0.set_aspect("equal"); ax0.axis("off")
ax0.add_patch(plt.Circle((0, 0), 0.4, fill=False, ec=RULE, lw=0.8))
ghost, = ax0.plot([0, 0.4], [0, 0], color=GREY, lw=1.5, ls="--")
arm, = ax0.plot([0, 0.4], [0, 0], color=INK, lw=4, solid_capstyle="round")
ax0.add_patch(plt.Circle((0, 0), 0.03, color=INK))
ax0.text(0, -0.47, "top view, joint axis vertical", ha="center", fontsize=6.5, color=GREY)
ax0.text(0.4, 0.45, "dashed: command", ha="right", fontsize=6.5, color=GREY)

ax1 = fig.add_axes([0.40, 0.55, 0.57, 0.28])
ax2 = fig.add_axes([0.40, 0.14, 0.57, 0.28])
for ax in (ax1, ax2):
    ax.set_xlim(0, T_END); ax.tick_params(labelsize=6.5, length=2)
ax1.set_ylim(-1.0, 1.0); ax1.set_ylabel("position, rad", fontsize=7); ax1.set_xticklabels([])
ax2.set_ylim(-1.1 * np.abs(tau).max(), 1.1 * np.abs(tau).max()); ax2.set_ylabel("estimated torque, N*m", fontsize=7)
ax2.set_xlabel("time, s (simulated, real time)", fontsize=7)
ax1.plot([w[0] for w in WAYPOINTS], [w[1] for w in WAYPOINTS], "o", color=GREY, ms=4, label="waypoints", zorder=1)
l_ref, = ax1.plot([], [], color=GREY, ls="--", label="command")
l_q, = ax1.plot([], [], color=INK, label="measured")
ax1.legend(loc="upper right", fontsize=6.5, frameon=False, ncol=3)
l_tau, = ax2.plot([], [], color=ACCENT)
clock = fig.text(0.02, 0.12, "", fontsize=8)


def make_frame(i):
    k = min(len(t) - 1, int(round(i / (N_FR - 1) * (len(t) - 1))))
    sl = slice(0, k + 1)
    l_ref.set_data(t[sl], qref[sl]); l_q.set_data(t[sl], q[sl]); l_tau.set_data(t[sl], tau[sl])
    arm.set_data([0, 0.4 * math.cos(q[k])], [0, 0.4 * math.sin(q[k])])
    ghost.set_data([0, 0.4 * math.cos(qref[k])], [0, 0.4 * math.sin(qref[k])])
    clock.set_text("t = %.2f s" % t[k])
    return fig


render_clip("ros2_trajectory", make_frame, N_FR, fps=FPS)
