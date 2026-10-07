"""Torque estimation and collision detection without a torque sensor, in the simulator.

SIMULATED, PRE-HARDWARE. Reproduces the collision case of the S_torque_sensing_R1 study on one
simulated S-S elbow (or S-D wrist): a forearm link with gravity, a contact-free multisine to set the
detection threshold, then the joint driven at 2 rad/s into a wall (stiff, and a soft human-arm-like
contact). The simulated unit has friction, efficiency and Kt drawn away from the nominal calibration
the estimator uses, plus current-sensor errors. Numbers printed here are simulation results under
stated assumptions, not product specifications.

Run: python examples/05_torque_estimation_collision.py [--unit S-D] [--no-ukf] [--plot out.png]
"""
import argparse
import math

import _path  # noqa: F401
import am_sdk as am
from am_sdk.estimation import (CollisionConfig, CollisionDetector, TorqueEstimator, UkfTorqueEstimator,
                               calibrate_threshold)
from am_sdk.estimation.simharness import (STUDY_S, EstimatorHarness, detection_latency, error_stats, lp1,
                                          study_calibration, study_truth_plant)

ap = argparse.ArgumentParser()
ap.add_argument("--unit", default="S-S", choices=sorted(STUDY_S))
ap.add_argument("--seed", type=int, default=20261006)
ap.add_argument("--no-ukf", action="store_true")
ap.add_argument("--plot", metavar="PNG", help="save a plot (needs matplotlib)")
args = ap.parse_args()

UNIT = args.unit
P = STUDY_S[UNIT]
DT = 1e-3
cal = study_calibration(UNIT)
plant = study_truth_plant(UNIT, args.seed)
J_TOT = cal.reflected_inertia_kgm2 + P["J_L"]
WALL = 0.4


def gravity(t, q, qd):
    return -P["mgl"] * math.cos(q)


def run(ref, q0, duration, wall_k=0.0, wall_c=0.0, threshold=math.inf, ukf=False):
    def contact(t, q, qd):
        pen = q - WALL
        return min(0.0, -wall_k * pen - wall_c * qd) if (wall_k > 0.0 and pen > 0.0) else 0.0

    def ext(t, q, qd):
        return gravity(t, q, qd) + contact(t, q, qd)
    opts = am.SimOptions(load_inertia_kgm2=P["J_L"], q0_rad=q0, ambient_C=45.0, external_torque=ext,
                         truth_plant=plant)
    s = am.am_init(am.AmSessionConfig(interface="sim:" + UNIT, cycle_us=1000, sim_options=[opts]))
    j = am.am_open_joint(s, 0)
    am.am_start(s)
    am.am_set_mode(j, am.AmMode.IMPEDANCE)
    am.am_enable(j, timeout_ms=1000)
    est = TorqueEstimator(cal)
    det = CollisionDetector(CollisionConfig(inertia_kgm2=J_TOT, gain_hz=30.0, threshold_Nm=threshold))
    h = EstimatorHarness(s, j, est, det, UkfTorqueEstimator(cal) if ukf else None, known_torque=gravity,
                         contact_torque=contact)
    t0 = am.am_read_telemetry(j).t_ns * 1e-9
    for k in range(int(duration / DT)):
        q, qd, qdd = ref(k * DT)
        tau_ff = P["mgl"] * math.cos(q) + J_TOT * qdd
        am.am_set_impedance(j, am.AmImpedance(q_ref_rad=q, qd_ref_rad_s=qd, kp_Nm_per_rad=P["Kp"],
                                              kd_Nm_s_per_rad=P["Kd"], tau_ff_Nm=tau_ff))
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


R = cal.rated_torque_Nm
print("%s, SIMULATED (S_torque_sensing_R1 assumptions, one seeded unit draw, seed %d)" % (UNIT, args.seed))
print("link: J %.3f kg m^2, m*g*l %.2f N*m, lever %.2f m (assumed)" % (P["J_L"], P["mgl"], P["lever_m"]))

# 1. contact-free multisine: estimate error and the detection threshold
free = run(multisine, 0.0, 10.0, ukf=not args.no_ukf)
skip = 200
truth = [s.truth_tau_out_Nm for s in free]
rows = [("baseline current model", [s.est.tau_out_Nm for s in free]),
        ("firmware open-loop (kt*iq*N*eta)", [s.firmware_tau_Nm for s in free])]
if not args.no_ukf:
    rows.append(("UKF (no learned residual)", [s.ukf_tau_Nm for s in free]))
print("\nfree motion, 10 s: estimate error vs simulator truth at the output flange (30 Hz band)")
t30 = lp1(truth, 30.0, DT)
for name, x in rows:
    st = error_stats([a - b for a, b in zip(lp1(x, 30.0, DT)[skip:], t30[skip:])], R)
    print("  %-34s rms %5.2f %%  p95 %5.2f %% of rated (%.3f N*m)" % (name, st["rms_pct_rated"], st["p95_pct_rated"],
                                                                     st["p95_Nm"]))
moving = [i for i, s in enumerate(free) if i >= skip and abs(s.est.qd_rad_s) > 0.1]
st = error_stats([free[i].est.tau_out_Nm - truth[i] for i in moving], R)
print("  baseline, moving samples only:     p95 %5.2f %% of rated (unfiltered, %d samples)" % (st["p95_pct_rated"], st["n"]))
stick = sum(1 for s in free[skip:] if s.est.quality & am.estimation.EST_STICK)
print("  quality: %d of %d samples flagged STICK" % (stick, len(free) - skip))
th = calibrate_threshold([s.residual_Nm for s in free[skip:]], 1.1)
print("\ncollision threshold = 1.1 x max |residual| in free motion = %.2f N*m (%.1f N at the hand)"
      % (th, th / P["lever_m"]))

# 2. collisions
results = {}
for case, kf in (("stiff wall", 1.0), ("soft contact", 0.15)):
    k = P["K_wall"] * kf
    c = 0.1 * math.sqrt(P["K_wall"]) * math.sqrt(kf)
    log = run(approach, -0.3, 1.2, wall_k=k, wall_c=c, threshold=th)
    d = detection_latency(log)
    results[case] = log
    if d and d["detected"]:
        print("%-12s contact K %.0f N*m/rad: detected %.0f ms after contact, contact %.2f N*m (%.1f N at the hand)"
              "%s" % (case, k, d["latency_s"] * 1e3, d["contact_at_detect_Nm"], d["contact_at_detect_Nm"] / P["lever_m"],
                      ", %d false alarm(s) before contact" % d["false_alarms_before_contact"]
                      if d["false_alarms_before_contact"] else ""))
    else:
        print("%-12s not detected" % case)

print("\nAll values: simulation of design assumptions. Targets to confirm on hardware: see docs/ESTIMATION.md.")

if args.plot:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; no plot")
    else:
        fig, ax = plt.subplots(3, 1, figsize=(9, 9))
        tt = [s.t_s for s in free]
        ax[0].plot(tt, truth, label="simulator truth", lw=1)
        ax[0].plot(tt, [s.est.tau_out_Nm for s in free], label="estimated (baseline)", lw=1)
        ax[0].set_ylabel("N*m")
        ax[0].set_title("%s free motion (SIMULATED)" % UNIT)
        ax[0].legend()
        for i, (case, log) in enumerate(results.items()):
            tt = [s.t_s for s in log]
            ax[i + 1].plot(tt, [-s.truth_contact_Nm for s in log], label="contact torque (truth)")
            ax[i + 1].plot(tt, [-s.residual_Nm for s in log], label="observer residual (estimated)")
            ax[i + 1].axhline(th, ls="--", c="k", lw=0.8, label="threshold")
            ax[i + 1].set_xlim(0.35, 0.6)
            ax[i + 1].set_ylabel("N*m")
            ax[i + 1].set_title(case + " (SIMULATED)")
            ax[i + 1].legend()
        ax[2].set_xlabel("s")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print("plot saved to", args.plot)
