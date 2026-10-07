"""Baseline current-model torque estimator.

SIMULATED, PRE-HARDWARE. Every output is an estimate, labelled as such, with an uncertainty bound
and a confidence value. No accuracy figure is published before the evaluation-unit dyno runs.

Model (output-referred, rigid gear, the same as c/src/am_est.c):

    tau_m    = N * Kt(T, iq) * iq                         motor torque before losses
    Kt(T,i)  = Kt_ref * (1 + a * (T - T_ref)) * (1 - k_sat * (i / i_sat)^2)
    f(w, T)  = (Tc + (Ts - Tc) * exp(-(w / ws)^2)) * tanh(w / w_eps) + b * w     (or the friction map)
    x        = tau_m - f(w, T) - J_arm * qdd               torque into the gear
    tau_out  = gear(x, w) - Tb * tanh(w / w_eps_b)         torque at the output flange
    gear(x)  = s * eta_f * x + (1 - s) * x / eta_r,  s = (1 + tanh(x * w / p_eps)) / 2

s blends driving (power flows to the load, x * eta_f) and back-driven operation (x / eta_r). At
standstill s = 1/2: the estimate sits in the middle of the stick band and the quality flag says so.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .calibration import EstimatorCalibration

TWO_PI = 2.0 * math.pi

# Quality bits (same values as AM_EST_Q_* in am_est.h).
EST_OK = 0
EST_STICK = 1 << 0            # joint at rest: the torque is only known to within the stick band
EST_AMBIGUOUS = 1 << 1        # Stribeck region or just after a reversal: friction level uncertain
EST_NO_TEMPERATURE = 1 << 2   # no temperature input: Kt at its reference temperature, wider bound
EST_SATURATED = 1 << 3        # current beyond the Kt saturation calibration point
EST_INVALID = 1 << 4          # inputs held, stale or not finite
EST_DITHER = 1 << 5           # dither active: the estimate is the dither-period average

_Q_NAMES = ((EST_STICK, "stick"), (EST_AMBIGUOUS, "ambiguous"), (EST_NO_TEMPERATURE, "no_temperature"),
            (EST_SATURATED, "saturated"), (EST_INVALID, "invalid"), (EST_DITHER, "dither"))

NO_TEMP_UNC_C = 40.0          # temperature uncertainty used when no temperature is given
DITHER_MAX_FRAC_RATED = 0.1   # dither amplitude cap, fraction of rated torque
DITHER_MAX_OF_BREAKAWAY = 1.5 # and multiple of the breakaway torque


def quality_str(q: int) -> str:
    if not q:
        return "ok"
    return "|".join(n for b, n in _Q_NAMES if q & b)


@dataclass
class EstimatorConfig:
    svf_hz: float = 40.0              # state-variable differentiator on the position (velocity, acceleration)
    svf_zeta: float = 0.8
    output_lp_hz: float = 100.0       # first-order filter on the estimate; 0 = off
    dither_lp_hz: float = 5.0         # filter while dither is active
    stick_vel_rad_s: float = 0.01     # |qd| below this: STICK
    ambiguous_vel_factor: float = 2.0 # |qd| below factor * Stribeck velocity: AMBIGUOUS
    reversal_hold_s: float = 0.02     # AMBIGUOUS for this long after leaving STICK
    w_eps_rad_s: float = 0.003        # friction sign regularisation
    power_eps_W: float = 0.02         # driving / back-driven blend
    bearing_w_eps_rad_s: float = 0.01
    confidence_band_frac_rated: float = 0.10   # confidence = 1 - bound / (this * rated)


DEFAULT_CONFIG = EstimatorConfig()


@dataclass
class TorqueEstimate:
    """Estimated (not measured) joint torque at the output flange, with its bound and quality."""
    tau_out_Nm: float = 0.0       # estimated torque delivered at the output flange
    tau_drive_Nm: float = 0.0     # the same without the armature inertia term (input of the collision observer)
    tau_motor_Nm: float = 0.0     # N * Kt(T) * iq
    friction_Nm: float = 0.0      # modelled friction at the current speed
    qd_rad_s: float = 0.0
    qdd_rad_s2: float = 0.0
    bound_Nm: float = 0.0         # model uncertainty half-width (about 2 sigma under the stated assumptions)
    confidence: float = 0.0       # 0..1, 1 - bound / (confidence band)
    quality: int = EST_OK
    label: str = "estimated"

    @property
    def quality_str(self) -> str:
        return quality_str(self.quality)


# ---------------------------------------------------------------------- pure model functions
def kt_at(cal: EstimatorCalibration, temp_C: float, iq_A: float) -> float:
    kt = cal.kt_Nm_per_A * (1.0 + cal.kt_temp_coeff_per_K * (temp_C - cal.kt_ref_temp_C))
    if cal.kt_sat_current_A > 0.0:
        x = iq_A / cal.kt_sat_current_A
        kt *= 1.0 - cal.kt_sat_frac * x * x
    return kt


def _interp(x: float, pts) -> float:
    if x <= pts[0][0]:
        return pts[0][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


def breakaway_at(cal: EstimatorCalibration, temp_C: float) -> float:
    ct = 1.0 + cal.coulomb_temp_coeff_per_K * (cal.friction_ref_temp_C - temp_C)
    if cal.friction_map:
        return cal.friction_map[0][1] * ct
    return cal.breakaway_Nm * ct


def friction_at(cal: EstimatorCalibration, w: float, temp_C: float, w_eps: float = DEFAULT_CONFIG.w_eps_rad_s) -> float:
    """Output-referred friction torque at speed w (sign of w)."""
    dT = cal.friction_ref_temp_C - temp_C
    ct = 1.0 + cal.coulomb_temp_coeff_per_K * dT
    sg = math.tanh(w / w_eps)
    if cal.friction_map:
        return _interp(abs(w), cal.friction_map) * ct * sg
    tc = cal.coulomb_Nm * ct
    ts = cal.breakaway_Nm * ct
    b = cal.viscous_Nm_s_per_rad * math.exp(cal.viscous_temp_exp_per_K * dT)
    x = w / cal.stribeck_vel_rad_s
    return (tc + (ts - tc) * math.exp(-x * x)) * sg + b * w


def efficiency_at(cal: EstimatorCalibration, temp_C: float, load_Nm: float) -> Tuple[float, float]:
    """(eta_forward, eta_reverse) at a temperature and |load|; scalars unless an efficiency map is set."""
    em = cal.efficiency_map
    if em is None:
        return cal.eta_forward, cal.eta_reverse

    def axis(v, xs):
        if len(xs) == 1 or v <= xs[0]:
            return 0, 0, 0.0
        if v >= xs[-1]:
            n = len(xs) - 1
            return n, n, 0.0
        for k in range(len(xs) - 1):
            if v <= xs[k + 1]:
                return k, k + 1, (v - xs[k]) / (xs[k + 1] - xs[k])
        n = len(xs) - 1
        return n, n, 0.0

    i0, i1, ft = axis(temp_C, em.temps_C)
    j0, j1, fl = axis(abs(load_Nm), em.loads_Nm)

    def bil(g):
        a = g[i0][j0] + (g[i0][j1] - g[i0][j0]) * fl
        b = g[i1][j0] + (g[i1][j1] - g[i1][j0]) * fl
        return a + (b - a) * ft
    return bil(em.eta_forward), bil(em.eta_reverse)


def gear_output(x: float, w: float, eta_f: float, eta_r: float, p_eps: float = DEFAULT_CONFIG.power_eps_W) -> float:
    s = 0.5 * (1.0 + math.tanh(x * w / p_eps))
    return s * x * eta_f + (1.0 - s) * x / eta_r


def stick_band_Nm(cal: EstimatorCalibration, tau_m: float, temp_C: float) -> float:
    """Half-width of the torque range consistent with a joint at rest: x * g = tau_m -/+ Ts for any
    g between eta_r and 1/eta_f."""
    ef, er = efficiency_at(cal, temp_C, abs(tau_m))
    ts = breakaway_at(cal, temp_C)
    return 0.5 * abs(tau_m) * (1.0 / er - ef) + 0.5 * ts * (1.0 / er + ef)


def current_model(cal: EstimatorCalibration, iq_A: float, qd_rad_s: float, qdd_rad_s2: float = 0.0,
                  temp_C: Optional[float] = None, cfg: EstimatorConfig = DEFAULT_CONFIG,
                  dither: bool = False) -> TorqueEstimate:
    """Pure function: one estimate from one sample. Quality here is the instantaneous part only
    (STICK, AMBIGUOUS, NO_TEMPERATURE, SATURATED, DITHER); TorqueEstimator adds reversal hold and filtering."""
    q = EST_OK
    unc_T = cal.unc_temp_C
    if temp_C is None or not math.isfinite(temp_C):
        temp_C = cal.kt_ref_temp_C
        unc_T = NO_TEMP_UNC_C
        q |= EST_NO_TEMPERATURE
    kt = kt_at(cal, temp_C, iq_A)
    tau_m = cal.gear_ratio * kt * iq_A
    f = friction_at(cal, qd_rad_s, temp_C, cfg.w_eps_rad_s)
    drive = tau_m - f
    ef, er = efficiency_at(cal, temp_C, abs(drive))
    brg = cal.bearing_Nm * math.tanh(qd_rad_s / cfg.bearing_w_eps_rad_s)
    j_term = cal.reflected_inertia_kgm2 * qdd_rad_s2
    tau_drive = gear_output(drive, qd_rad_s, ef, er, cfg.power_eps_W) - brg
    tau_out = gear_output(drive - j_term, qd_rad_s, ef, er, cfg.power_eps_W) - brg
    if cal.kt_sat_current_A > 0.0 and abs(iq_A) > cal.kt_sat_current_A:
        q |= EST_SATURATED
    aw = abs(qd_rad_s)
    if aw < cfg.stick_vel_rad_s:
        q |= EST_DITHER if dither else EST_STICK
    elif aw < cfg.ambiguous_vel_factor * cal.stribeck_vel_rad_s and not cal.friction_map:
        q |= EST_AMBIGUOUS
    bound = _bound(cal, tau_m, f, tau_out, j_term, temp_C, unc_T, q)
    return TorqueEstimate(tau_out_Nm=tau_out, tau_drive_Nm=tau_drive, tau_motor_Nm=tau_m, friction_Nm=f,
                          qd_rad_s=qd_rad_s, qdd_rad_s2=qdd_rad_s2, bound_Nm=bound,
                          confidence=_confidence(cal, cfg, bound), quality=q)


def _bound(cal, tau_m, f, tau_out, j_term, temp_C, unc_T, q) -> float:
    a = cal.unc_kt_rel * tau_m
    b = cal.kt_temp_coeff_per_K * unc_T * tau_m
    c = cal.unc_friction_rel * f
    d = cal.unc_eta_abs * tau_out
    e = cal.unc_inertia_rel * j_term
    mov = math.sqrt(a * a + b * b + c * c + d * d + e * e)
    if q & EST_STICK:
        return mov + stick_band_Nm(cal, tau_m, temp_C)
    if q & EST_DITHER:
        return mov + cal.unc_friction_rel * breakaway_at(cal, temp_C)
    if q & EST_AMBIGUOUS:
        return mov + 0.5 * stick_band_Nm(cal, tau_m, temp_C)
    return mov


def _confidence(cal, cfg, bound) -> float:
    band = cfg.confidence_band_frac_rated * cal.rated_torque_Nm
    return max(0.0, min(1.0, 1.0 - bound / band)) if band > 0 else 0.0


# ---------------------------------------------------------------------- dither helper
def dither_amplitude_max(cal: EstimatorCalibration) -> float:
    """Largest dither amplitude the SDK accepts: min(1.5 x breakaway, 10 % of rated)."""
    return min(DITHER_MAX_OF_BREAKAWAY * cal.breakaway_Nm, DITHER_MAX_FRAC_RATED * cal.rated_torque_Nm)


def dither_torque(cal: EstimatorCalibration, amplitude_Nm: float, freq_Hz: float, t_s: float) -> float:
    """Optional bounded dither (off unless you add it to your torque feed-forward). It keeps the gear
    out of stick so a static load can be estimated; it costs a small vibration and heat."""
    a = max(0.0, min(amplitude_Nm, dither_amplitude_max(cal)))
    return a * math.sin(TWO_PI * freq_Hz * t_s)


# ---------------------------------------------------------------------- stateful estimator
class TorqueEstimator:
    """Per-joint estimator for a 1 kHz host loop: state-variable differentiator on the output
    position, the current model, quality flags with a reversal hold, and an output filter."""

    def __init__(self, cal: EstimatorCalibration, cfg: Optional[EstimatorConfig] = None):
        self.cal = cal
        self.cfg = cfg or EstimatorConfig()
        self.reset()

    def reset(self, q_rad: Optional[float] = None):
        self._init = q_rad is not None
        self.p = q_rad if q_rad is not None else 0.0
        self.v = 0.0
        self.a = 0.0
        self.tau_f = 0.0
        self.tau_d = 0.0
        self.since_stick = math.inf
        self.was_stick = False
        self._last_t_ns = None

    def update(self, dt: float, q_rad: float, iq_A: float, temp_C: Optional[float] = None,
               dither_active: bool = False, saturated: bool = False, valid: bool = True) -> TorqueEstimate:
        cfg = self.cfg
        finite = all(math.isfinite(x) for x in (dt, q_rad, iq_A)) and dt > 0.0
        if not finite:
            return TorqueEstimate(tau_out_Nm=self.tau_f, tau_drive_Nm=self.tau_d, qd_rad_s=self.v,
                                  qdd_rad_s2=self.a, bound_Nm=math.inf, confidence=0.0, quality=EST_INVALID)
        if not self._init:
            self.p, self.v, self.a, self._init = q_rad, 0.0, 0.0, True
            first = True
        else:
            first = False
        wn = TWO_PI * cfg.svf_hz
        self.a = wn * wn * (q_rad - self.p) - 2.0 * cfg.svf_zeta * wn * self.v
        self.v += self.a * dt
        self.p += self.v * dt
        est = current_model(self.cal, iq_A, self.v, self.a, temp_C, cfg, dither_active)
        stick = bool(est.quality & (EST_STICK | EST_DITHER))
        if stick:
            self.since_stick = 0.0
        else:
            self.since_stick += dt
            if self.since_stick < cfg.reversal_hold_s and not (est.quality & EST_AMBIGUOUS):
                t_used = temp_C if (temp_C is not None and math.isfinite(temp_C)) else self.cal.kt_ref_temp_C
                est.quality |= EST_AMBIGUOUS
                est.bound_Nm += 0.5 * stick_band_Nm(self.cal, est.tau_motor_Nm, t_used)
                est.confidence = _confidence(self.cal, cfg, est.bound_Nm)
        fc = cfg.dither_lp_hz if dither_active else cfg.output_lp_hz
        if first or fc <= 0.0:
            self.tau_f, self.tau_d = est.tau_out_Nm, est.tau_drive_Nm
        else:
            k = 1.0 - math.exp(-TWO_PI * fc * dt)
            self.tau_f += k * (est.tau_out_Nm - self.tau_f)
            self.tau_d += k * (est.tau_drive_Nm - self.tau_d)
        est.tau_out_Nm, est.tau_drive_Nm = self.tau_f, self.tau_d
        if saturated:
            est.quality |= EST_SATURATED
        if not valid:
            est.quality |= EST_INVALID
            est.confidence = 0.0
        return est

    def update_from_telemetry(self, tlm, dt: float = 1e-3, dither_active: bool = False) -> TorqueEstimate:
        """Feed one am_read_telemetry() sample. Temperature: the magnet model temperature when the
        extended telemetry is fresh, else the winding temperature, else none (flagged)."""
        if self._last_t_ns is not None and tlm.t_ns > self._last_t_ns:
            dt = (tlm.t_ns - self._last_t_ns) * 1e-9
        self._last_t_ns = tlm.t_ns
        temp = None
        if tlm.extended_valid:
            if tlm.t_magnet_C:
                temp = tlm.t_magnet_C
            elif tlm.t_winding_C:
                temp = tlm.t_winding_C
        return self.update(dt, tlm.q_rad, tlm.iq_A, temp, dither_active=dither_active)
