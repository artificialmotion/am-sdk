# SPDX-License-Identifier: Apache-2.0
"""Run the estimators against the simulator: ground truth next to the estimate, collision events.

SIMULATED, PRE-HARDWARE. The ground truth is the simulator's own plant (SimJoint.truth); on hardware
there is no truth signal and only the estimator part of this harness applies.

    s = am.am_init(am.AmSessionConfig(interface="sim:S-S", sim_options=[am.SimOptions(truth_plant=plant)]))
    ...
    h = EstimatorHarness(s, joint, TorqueEstimator(cal), CollisionDetector(cfg), known_torque=gravity)
    for k in range(n):
        <set your command>
        sample = h.step()

study_truth_plant() gives the plant assumptions of the S_torque_sensing_R1 study (S-S, S-D) with a
seeded unit-to-unit and field-drift draw.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, List, Optional

from ..api import am_cycle, am_read_telemetry
from ..backends.sim import TruthPlant
from .calibration import EstimatorCalibration
from .collision import CollisionDetector
from .core import TorqueEstimate, TorqueEstimator
from .ukf import UkfTorqueEstimator

TorqueFn = Callable[[float, float, float], float]

# S_torque_sensing_R1 plant assumptions (output-referred). ESTIMATE / ASSUMED values from that study:
# breakaway = the published backdrive torque, Coulomb, Stribeck velocity, viscous, efficiencies,
# output bearing; link: inertia, gravity m*g*l, collision lever and speed. Not unit ratings.
STUDY_S = {
    "S-S": dict(Tc=0.45, Ts=0.65, ws=0.05, b=0.04, eta_f=0.935, eta_r=0.861, Tb=0.04, J_L=0.06, mgl=3.5,
                Kp=150.0, Kd=3.5, K_wall=3000.0, lever_m=0.30, v_coll=2.0, Imax=22.0),
    "S-D": dict(Tc=0.25, Ts=0.36, ws=0.08, b=0.012, eta_f=0.965, eta_r=0.925, Tb=0.03, J_L=0.006, mgl=0.6,
                Kp=30.0, Kd=0.8, K_wall=600.0, lever_m=0.10, v_coll=3.0, Imax=19.0),
}


def study_calibration(unit: str) -> EstimatorCalibration:
    """Nominal (family-level) calibration matching the study's nominal friction and efficiency."""
    p = STUDY_S[unit]
    cal = EstimatorCalibration.from_unit(unit)
    cal.data_basis = "simulated"
    cal.coulomb_Nm, cal.breakaway_Nm, cal.stribeck_vel_rad_s = p["Tc"], p["Ts"], p["ws"]
    cal.viscous_Nm_s_per_rad, cal.eta_forward, cal.eta_reverse, cal.bearing_Nm = p["b"], p["eta_f"], p["eta_r"], p["Tb"]
    cal.note = "nominal S_torque_sensing_R1 assumptions (simulated)"
    return cal


def study_truth_plant(unit: str, seed: int = 20261006) -> TruthPlant:
    """One simulated unit: the study's nominal plant with a seeded draw of EOL spread and field drift
    (Kt +-2 %, friction log-normal 30 %, efficiency loss 15 %, drift 10-12 %, current sensor errors,
    temperature estimate bias). The estimator calibration stays nominal, so these are the errors it sees."""
    p = STUDY_S[unit]
    r = random.Random(seed)

    def ln(s):
        return math.exp(r.gauss(0.0, s))
    drift_tc, drift_b, drift_eta = 1 + r.gauss(0, 0.12), 1 + r.gauss(0, 0.12), 1 + r.gauss(0, 0.10)
    tc = p["Tc"] * ln(0.3)
    ts = tc * p["Ts"] / p["Tc"] * ln(0.1)
    eta_f = 1 - (1 - p["eta_f"]) * ln(0.15) * drift_eta
    eta_r = 1 - (1 - p["eta_r"]) * ln(0.15) * drift_eta
    return TruthPlant(kt_scale=1 + r.gauss(0, 0.02), kt_sat_frac=r.uniform(0.03, 0.08), kt_sat_current_A=p["Imax"],
                      coulomb_Nm=tc * drift_tc, breakaway_Nm=ts * drift_tc, stribeck_vel_rad_s=p["ws"] * ln(0.3),
                      viscous_Nm_s_per_rad=p["b"] * ln(0.3) * drift_b, coulomb_temp_coeff_per_K=r.uniform(0.002, 0.006),
                      viscous_temp_exp_per_K=r.uniform(0.015, 0.03), eta_forward=eta_f, eta_reverse=eta_r,
                      bearing_Nm=p["Tb"] * ln(0.3), friction_ripple=r.uniform(0.05, 0.15),
                      torque_ripple=r.uniform(0.005, 0.015), i_gain_err=r.gauss(0, 0.003),
                      i_offset_A=r.gauss(0, 0.015), i_noise_A=0.03, temp_sensor_bias_C=r.gauss(0, 2.5), seed=seed)


@dataclass
class HarnessSample:
    t_s: float
    q_rad: float
    truth_tau_out_Nm: float        # simulator ground truth at the output flange
    truth_contact_Nm: float        # external torque not explained by known_torque (the contact)
    est: TorqueEstimate            # baseline estimate (labelled "estimated")
    firmware_tau_Nm: float         # telemetry tau_est_Nm: open-loop kt*iq*N*eta, no temperature or friction terms
    ukf_tau_Nm: float              # NaN without a UKF
    residual_Nm: float             # collision observer residual (NaN without a detector)
    collision: bool
    onset: bool


class EstimatorHarness:
    def __init__(self, session, joint, estimator: TorqueEstimator, detector: Optional[CollisionDetector] = None,
                 ukf: Optional[UkfTorqueEstimator] = None, known_torque: Optional[TorqueFn] = None,
                 contact_torque: Optional[TorqueFn] = None, record: bool = True):
        """known_torque(t, q, qd): your model of the link (gravity...), used by the detector.
        contact_torque(t, q, qd): the simulated contact for the log; default: the external torque minus
        known_torque (then the end-of-cycle state is used, so it is not exactly zero without contact)."""
        self.s, self.j = session, joint
        self.sim = joint.jb
        if not hasattr(self.sim, "truth"):
            raise TypeError("EstimatorHarness needs a simulator joint (interface 'sim:...')")
        self.est, self.det, self.ukf = estimator, detector, ukf
        self.known = known_torque
        self.contact = contact_torque
        self.record = record
        self.dither_active = False     # set True while you add dither_torque() to the command
        self.log: List[HarnessSample] = []

    def step(self) -> HarnessSample:
        am_cycle(self.s)
        tlm = am_read_telemetry(self.j)
        dt = self.s.dt
        t = tlm.t_ns * 1e-9
        e = self.est.update_from_telemetry(tlm, dt, dither_active=self.dither_active)
        known = self.known(t, tlm.q_rad, tlm.qd_rad_s) if self.known else 0.0
        tr = self.sim.truth
        if self.contact is not None:
            contact = self.contact(t, self.sim.q, self.sim.qd)
        else:
            contact = tr.tau_ext_Nm - (self.known(t, self.sim.q, self.sim.qd) if self.known else 0.0)
        ukf_tau = math.nan
        if self.ukf is not None:
            temp = tlm.t_magnet_C if tlm.extended_valid else None
            ukf_tau = self.ukf.update(dt, tlm.q_rad, tlm.iq_A, temp).tau_out_Nm
        r, active, onset = math.nan, False, False
        if self.det is not None:
            # momentum from the drive's velocity (0x606C): less lag than the estimator's 40 Hz differentiator
            o = self.det.update(dt, tlm.qd_rad_s, e.tau_drive_Nm, known, e.quality)
            r, active, onset = o.residual_Nm, o.active, o.onset
        smp = HarnessSample(t_s=t, q_rad=tlm.q_rad, truth_tau_out_Nm=tr.tau_out_Nm,
                            truth_contact_Nm=contact, est=e, firmware_tau_Nm=tlm.tau_est_Nm,
                            ukf_tau_Nm=ukf_tau, residual_Nm=r, collision=active, onset=onset)
        if self.record:
            self.log.append(smp)
        return smp


# ---------------------------------------------------------------------- statistics helpers
def lp1(xs: List[float], fc_hz: float, dt: float) -> List[float]:
    a = math.exp(-2.0 * math.pi * fc_hz * dt)
    out, y = [], None
    for x in xs:
        y = x if y is None else a * y + (1.0 - a) * x
        out.append(y)
    return out


def error_stats(err: List[float], rated_Nm: float) -> dict:
    if not err:
        return {"n": 0}
    a = sorted(abs(e) for e in err)
    p95 = a[min(len(a) - 1, int(math.ceil(0.95 * len(a))) - 1)]
    rms = math.sqrt(sum(e * e for e in err) / len(err))
    return {"n": len(err), "rms_Nm": rms, "p95_Nm": p95, "max_Nm": a[-1], "bias_Nm": sum(err) / len(err),
            "rms_pct_rated": 100.0 * rms / rated_Nm, "p95_pct_rated": 100.0 * p95 / rated_Nm}


def detection_latency(log: List[HarnessSample], contact_eps_Nm: float = 1e-6) -> Optional[dict]:
    """First contact (|truth contact| > eps) and the first detection at or after it."""
    i0 = next((i for i, s in enumerate(log) if abs(s.truth_contact_Nm) > contact_eps_Nm), None)
    if i0 is None:
        return None
    false_alarms = sum(1 for s in log[:i0] if s.onset)
    i1 = next((i for i in range(i0, len(log)) if log[i].collision), None)
    if i1 is None:
        return {"contact_t_s": log[i0].t_s, "detected": False, "false_alarms_before_contact": false_alarms}
    return {"contact_t_s": log[i0].t_s, "detected": True, "latency_s": log[i1].t_s - log[i0].t_s,
            "contact_at_detect_Nm": abs(log[i1].truth_contact_Nm), "residual_at_detect_Nm": log[i1].residual_Nm,
            "false_alarms_before_contact": false_alarms}
