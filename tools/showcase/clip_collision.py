"""Clip A: torque estimate vs simulator truth and the collision flag (S-S elbow into a stiff wall). SIMULATED.

Re-runs the stiff-wall case of python/examples/05_torque_estimation_collision.py (same seed, same plant
assumptions) and animates it. Output: docs/media/collision_detection.{mp4,gif}
"""
import math

from common import *  # noqa: F401,F403
import am_sdk as am
from am_sdk.estimation import CollisionConfig, CollisionDetector, TorqueEstimator, calibrate_threshold
from am_sdk.estimation.simharness import STUDY_S, EstimatorHarness, detection_latency, study_calibration, study_truth_plant

UNIT, SEED, DT, WALL = "S-S", 20261006, 1e-3, 0.4
P = STUDY_S[UNIT]
cal = study_calibration(UNIT)
plant = study_truth_plant(UNIT, SEED)
J_TOT = cal.reflected_inertia_kgm2 + P["J_L"]


def gravity(t, q, qd):
    return -P["mgl"] * math.cos(q)


def run(ref, q0, duration, wall_k=0.0, wall_c=0.0, threshold=math.inf):
    def contact(t, q, qd):
        pen = q - WALL
        return min(0.0, -wall_k * pen - wall_c * qd) if (wall_k > 0.0 and pen > 0.0) else 0.0

    opts = am.SimOptions(load_inertia_kgm2=P["J_L"], q0_rad=q0, ambient_C=45.0,
                         external_torque=lambda t, q, qd: gravity(t, q, qd) + contact(t, q, qd), truth_plant=plant)
    s = am.am_init(am.AmSessionConfig(interface="sim:" + UNIT, cycle_us=1000, sim_options=[opts]))
    j = am.am_open_joint(s, 0)
    am.am_start(s)
    am.am_set_mode(j, am.AmMode.IMPEDANCE)
    am.am_enable(j, timeout_ms=1000)
    det = CollisionDetector(CollisionConfig(inertia_kgm2=J_TOT, gain_hz=30.0, threshold_Nm=threshold))
    h = EstimatorHarness(s, j, TorqueEstimator(cal), det, None, known_torque=gravity, contact_torque=contact)
    t0 = am.am_read_telemetry(j).t_ns * 1e-9
    for k in range(int(duration / DT)):
        q, qd, qdd = ref(k * DT)
        am.am_set_impedance(j, am.AmImpedance(q_ref_rad=q, qd_ref_rad_s=qd, kp_Nm_per_rad=P["Kp"],
                                              kd_Nm_s_per_rad=P["Kd"], tau_ff_Nm=P["mgl"] * math.cos(q) + J_TOT * qdd))
        h.step()
    am.am_deinit(s)
    for smp in h.log:
        smp.t_s -= t0
    return h.log


A, F, PH = [0.5, 0.3, 0.15, 0.08], [0.25, 0.6, 1.3, 2.5], [0.3, 1.9, 4.0, 5.2]


def multisine(t):
    w = [2 * math.pi * f for f in F]
    return (sum(a * (math.sin(wi * t + p) - math.sin(p)) for a, wi, p in zip(A, w, PH)),
            sum(a * wi * math.cos(wi * t + p) for a, wi, p in zip(A, w, PH)),
            -sum(a * wi * wi * math.sin(wi * t + p) for a, wi, p in zip(A, w, PH)))


def approach(t, v=P["v_coll"], ta=0.15):
    if t < ta:
        return -0.3 + 0.5 * v / ta * t * t, v * t / ta, v / ta
    return -0.3 + 0.5 * v * ta + v * (t - ta), v, 0.0


free = run(multisine, 0.0, 10.0)
th = calibrate_threshold([s.residual_Nm for s in free[200:]], 1.1)
log = run(approach, -0.3, 1.2, wall_k=P["K_wall"], wall_c=0.1 * math.sqrt(P["K_wall"]), threshold=th)
d = detection_latency(log)
print("threshold %.2f N m, detected after %.0f ms" % (th, d["latency_s"] * 1e3))

T0, T1 = 0.25, 0.75
S = [s for s in log if T0 <= s.t_s <= T1]
t = np.array([s.t_s for s in S])
q = np.array([s.q_rad for s in S])
true_tau = np.array([s.truth_tau_out_Nm for s in S])
est_tau = np.array([s.est.tau_out_Nm for s in S])
c_true = np.array([-s.truth_contact_Nm for s in S])
c_res = np.array([-s.residual_Nm for s in S])
flag = np.array([1.0 if s.collision else 0.0 for s in S])
t_contact = d["contact_t_s"]
t_detect = t_contact + d["latency_s"]

N_FR = 225            # 9 s at 25 fps for 0.5 s of simulated time: slow motion x18
SLOW = (T1 - T0) / (N_FR / 25.0)
fig = new_fig()
title(fig, "Collision detection without a torque sensor",
      "%s elbow, motor current and encoder only. Stiff wall at %.0f N*m/rad." % (UNIT, P["K_wall"]))
caption(fig, "Simulated. One seeded unit draw, design assumptions (see docs/ESTIMATION.md). Slow motion x%.0f." % (1 / SLOW))

# schematic
ax0 = fig.add_axes([0.02, 0.10, 0.30, 0.72])
ax0.set_xlim(-0.12, 0.42)
ax0.set_ylim(-0.22, 0.32)
ax0.set_aspect("equal")
ax0.axis("off")
L = P["lever_m"]
wa = WALL
wall_c = np.array([(L + 0.02) * math.cos(wa), (L + 0.02) * math.sin(wa)])
tdir = np.array([-math.sin(wa), math.cos(wa)])
ax0.plot(*zip(wall_c - 0.17 * tdir, wall_c + 0.17 * tdir), color=INK, lw=5, solid_capstyle="butt")
ax0.plot([-0.03, 0.0], [0, 0], color=INK, lw=0)
ax0.add_patch(plt.Circle((0, 0), 0.015, color=INK))
link, = ax0.plot([0, L], [0, 0], color=INK, lw=3, solid_capstyle="round")
tip, = ax0.plot([L], [0], "o", color=INK, ms=5)
flagtxt = ax0.text(0.15, -0.17, "", ha="center", fontsize=9, color=ACCENT)
ax0.text(0.15, 0.29, "t = ", ha="center", fontsize=7, color=GREY)
ttxt = ax0.text(0.17, 0.29, "", ha="left", fontsize=7, color=GREY)

def style(ax, ylabel, ylim):
    ax.set_xlim(T0, T1)
    ax.set_ylim(*ylim)
    ax.set_ylabel(ylabel, fontsize=7)
    ax.tick_params(labelsize=6.5, length=2)
    ax.axvline(t_contact, color=RULE, lw=0.8, zorder=0)

ax1 = fig.add_axes([0.40, 0.62, 0.57, 0.22])
ax2 = fig.add_axes([0.40, 0.33, 0.57, 0.22])
ax3 = fig.add_axes([0.40, 0.13, 0.57, 0.12])
style(ax1, "joint torque, N*m", (min(true_tau.min(), est_tau.min()) - 1, max(true_tau.max(), est_tau.max()) * 1.12))
style(ax2, "contact torque, N*m", (-3, max(c_true.max(), c_res.max()) * 1.12))
style(ax3, "", (-0.1, 1.2))
ax3.set_yticks([0, 1])
ax3.set_yticklabels(["off", "on"])
ax3.set_xlabel("time, s (simulated)", fontsize=7)
ax1.set_xticklabels([])
ax2.set_xticklabels([])
l1a, = ax1.plot([], [], color=INK, label="truth (simulator)")
l1b, = ax1.plot([], [], color=ACCENT, lw=1.1, label="estimated")
ax1.legend(loc="upper left", fontsize=6.5, frameon=False, ncol=1)
ax2.axhline(th, color=GREY, ls="--", lw=0.8)
ax2.text(T1, th, " threshold", fontsize=6, color=GREY, va="bottom", ha="right")
l2a, = ax2.plot([], [], color=INK, label="contact (truth)")
l2b, = ax2.plot([], [], color=ACCENT, lw=1.1, label="observer residual")
ax2.legend(loc="upper left", fontsize=6.5, frameon=False, ncol=1)
l3, = ax3.step([], [], where="post", color=ACCENT)
ax3.text(T0 + 0.005, 1.0, "contact flag", fontsize=6.5, va="top", color=INK)
dt_txt = fig.text(0.97, 0.90, "", ha="right", fontsize=7.5, color=INK)


def make_frame(i):
    k = min(len(t) - 1, int(round(i / (N_FR - 1) * (len(t) - 1))))
    sl = slice(0, k + 1)
    l1a.set_data(t[sl], true_tau[sl]); l1b.set_data(t[sl], est_tau[sl])
    l2a.set_data(t[sl], c_true[sl]); l2b.set_data(t[sl], c_res[sl])
    l3.set_data(t[sl], flag[sl])
    a = q[k]
    link.set_data([0, L * math.cos(a)], [0, L * math.sin(a)])
    tip.set_data([L * math.cos(a)], [L * math.sin(a)])
    flagtxt.set_text("contact detected" if flag[k] else "")
    ttxt.set_text("%.3f s" % t[k])
    dt_txt.set_text("detected %.0f ms after contact" % (d["latency_s"] * 1e3) if t[k] >= t_detect else "")
    return fig


render_clip("collision_detection", make_frame, N_FR)
