"""Optional UKF torque estimator and learned-residual hooks (standard library only).

SIMULATED, PRE-HARDWARE. Adopt the UKF or a learned residual on a joint only where it beats the
baseline current model on dyno data with a reference torque transducer (adoption_check). In the
S_torque_sensing_R1 simulation it helped while moving and did not help at standstill; the
simulation cannot show the learning benefit, so that decision waits for the evaluation-unit dyno.

UKF state x = [theta, omega, tau_t] (output-referred angle, speed, torque out of the gear).
Process: J_arm * omega' = tau_m - loss(omega, tau_t, T, iq),  tau_t a random walk.
loss = f(omega, T) + g(tau_t, omega) * tau_t + residual(omega, tau_t, T, iq),
g = s / eta_f + (1 - s) * eta_r with s = (1 + tanh(tau_t * omega / p_eps)) / 2.
Measurement: the output angle from the motor encoder (quantisation noise).
Output: tau_out = tau_t - Tb * tanh(omega / w_eps_b).

A residual model is any callable residual(omega, tau_t, T, iq) -> N*m of extra loss. It is evaluated
once per update at the prior mean (cheap enough for a 1 kHz Python loop).
"""
from __future__ import annotations

import math
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from .calibration import EstimatorCalibration
from .core import (DEFAULT_CONFIG, EST_NO_TEMPERATURE, EST_OK, EST_STICK, EstimatorConfig,
                   TorqueEstimate, _confidence, efficiency_at, friction_at, kt_at)

Residual = Callable[[float, float, float, float], float]


def baseline_loss(cal: EstimatorCalibration, w: float, tau_t: float, temp_C: float,
                  cfg: EstimatorConfig = DEFAULT_CONFIG) -> float:
    """Motor-side torque (output-referred) needed to hold speed w while the gear delivers tau_t."""
    ef, er = efficiency_at(cal, temp_C, abs(tau_t))
    s = 0.5 * (1.0 + math.tanh(tau_t * w / cfg.power_eps_W))
    g = s / ef + (1.0 - s) * er
    return friction_at(cal, w, temp_C, cfg.w_eps_rad_s) + g * tau_t


# ---------------------------------------------------------------------- tiny 3x3 linear algebra
def _chol3(a):
    l00 = math.sqrt(a[0][0])
    l10 = a[1][0] / l00
    l20 = a[2][0] / l00
    l11 = math.sqrt(a[1][1] - l10 * l10)
    l21 = (a[2][1] - l20 * l10) / l11
    l22 = math.sqrt(a[2][2] - l20 * l20 - l21 * l21)
    return ((l00, 0.0, 0.0), (l10, l11, 0.0), (l20, l21, l22))


class UkfTorqueEstimator:
    def __init__(self, cal: EstimatorCalibration, cfg: Optional[EstimatorConfig] = None,
                 residual: Optional[Residual] = None, q_tau_frac_rated: float = 0.01,
                 unmodelled_Nm: float = 0.05, encoder_bits: int = 19, substeps: int = 4):
        self.cal = cal
        self.cfg = cfg or EstimatorConfig()
        self.residual = residual
        self.q_tau = (q_tau_frac_rated * cal.rated_torque_Nm) ** 2
        self.unmodelled = unmodelled_Nm
        lsb = 2.0 * math.pi / (1 << encoder_bits) / cal.gear_ratio
        self.r_meas = lsb * lsb / 12.0 + 1e-12
        self.substeps = substeps
        n, alpha, beta, kappa = 3, 0.5, 2.0, 0.0
        self.lam = alpha * alpha * (n + kappa) - n
        self.c = n + self.lam
        self.wm = [self.lam / self.c] + [0.5 / self.c] * (2 * n)
        self.wc = [self.wm[0] + (1.0 - alpha * alpha + beta)] + [0.5 / self.c] * (2 * n)
        self.x = None
        self.P = None

    def reset(self, q_rad: Optional[float] = None):
        self.x = None if q_rad is None else [q_rad, 0.0, 0.0]
        if q_rad is not None:
            self._p_init()

    def _p_init(self):
        self.P = [[1e-8, 0.0, 0.0], [0.0, 1e-2, 0.0], [0.0, 0.0, (0.3 * self.cal.rated_torque_Nm) ** 2]]

    def update(self, dt: float, q_rad: float, iq_A: float, temp_C: Optional[float] = None) -> TorqueEstimate:
        cal, cfg = self.cal, self.cfg
        quality = EST_OK
        if temp_C is None or not math.isfinite(temp_C):
            temp_C = cal.kt_ref_temp_C
            quality |= EST_NO_TEMPERATURE
        if self.x is None:
            self.x = [q_rad, 0.0, 0.0]
            self._p_init()
        J = cal.reflected_inertia_kgm2
        tau_m = cal.gear_ratio * kt_at(cal, temp_C, iq_A) * iq_A
        res = self.residual(self.x[1], self.x[2], temp_C, iq_A) if self.residual else 0.0
        # sigma points
        P = self.P
        try:
            L = _chol3([[self.c * P[i][j] for j in range(3)] for i in range(3)])
        except (ValueError, ZeroDivisionError):
            P = [[P[i][j] + (1e-10 if i == j else 0.0) for j in range(3)] for i in range(3)]
            L = _chol3([[self.c * P[i][j] for j in range(3)] for i in range(3)])
        x = self.x
        sig = [list(x)]
        for k in range(3):
            sig.append([x[i] + L[i][k] for i in range(3)])
        for k in range(3):
            sig.append([x[i] - L[i][k] for i in range(3)])
        h = dt / self.substeps
        for s in sig:
            for _ in range(self.substeps):
                acc = (tau_m - baseline_loss(cal, s[1], s[2], temp_C, cfg) - res) / J
                s[0] += s[1] * h
                s[1] += acc * h
        xm = [sum(self.wm[k] * sig[k][i] for k in range(7)) for i in range(3)]
        q_w = (self.unmodelled / J * dt) ** 2
        Q = (1e-14, q_w, self.q_tau)
        Pm = [[0.0] * 3 for _ in range(3)]
        for k in range(7):
            d = [sig[k][i] - xm[i] for i in range(3)]
            for i in range(3):
                for j in range(3):
                    Pm[i][j] += self.wc[k] * d[i] * d[j]
        for i in range(3):
            Pm[i][i] += Q[i]
        zm = xm[0]
        S = self.r_meas
        C = [0.0, 0.0, 0.0]
        for k in range(7):
            dz = sig[k][0] - zm
            S += self.wc[k] * dz * dz
            for i in range(3):
                C[i] += self.wc[k] * (sig[k][i] - xm[i]) * dz
        K = [C[i] / S for i in range(3)]
        innov = q_rad - zm
        self.x = [xm[i] + K[i] * innov for i in range(3)]
        Pn = [[Pm[i][j] - K[i] * K[j] * S for j in range(3)] for i in range(3)]
        self.P = [[0.5 * (Pn[i][j] + Pn[j][i]) for j in range(3)] for i in range(3)]
        w, tau_t = self.x[1], self.x[2]
        tau_out = tau_t - cal.bearing_Nm * math.tanh(w / cfg.bearing_w_eps_rad_s)
        if abs(w) < cfg.stick_vel_rad_s:
            quality |= EST_STICK
        bound = 2.0 * math.sqrt(max(self.P[2][2], 0.0))
        return TorqueEstimate(tau_out_Nm=tau_out, tau_drive_Nm=tau_out, tau_motor_Nm=tau_m,
                              friction_Nm=friction_at(cal, w, temp_C, cfg.w_eps_rad_s), qd_rad_s=w,
                              bound_Nm=bound, confidence=_confidence(cal, cfg, bound), quality=quality,
                              label="estimated (UKF)")


# ---------------------------------------------------------------------- learned residual (linear, physics-structured)
def residual_features(w: float, tau: float, temp_C: float, iq: float) -> List[float]:
    """Physics-structured basis from the S_torque_sensing_R1 study (Coulomb/Stribeck shapes, viscous with
    temperature, load-dependent mesh loss forward/reverse, saturation, offsets)."""
    T = temp_C - 40.0
    sg = math.tanh(w / 0.003)
    fwd = 0.5 * (1.0 + math.tanh(tau * w / 0.02))
    at = abs(tau)
    return [sg, sg * math.exp(-(w / 0.05) ** 2), sg * math.exp(-(w / 0.2) ** 2), w, w * T, w * T * T, sg * T,
            at * sg * fwd, at * sg * (1.0 - fwd), at * sg * T, tau, tau * abs(iq), tau * iq * iq, tau * T, 1.0, T]


def _solve(A: List[List[float]], b: List[float]) -> List[float]:
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-300:
            raise ValueError("singular system")
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            if f:
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for c in range(n - 1, -1, -1):
        x[c] = (M[c][n] - sum(M[c][k] * x[k] for k in range(c + 1, n))) / M[c][c]
    return x


Sample = Tuple[float, float, float, float, float]   # (w, tau_ref at the flange, temp_C, iq, y residual target)


def residual_targets(cal: EstimatorCalibration, points: Iterable[Tuple[float, float, float, float]],
                     cfg: EstimatorConfig = DEFAULT_CONFIG) -> List[Sample]:
    """Dyno steady-speed points (w, tau_ref, temp_C, iq) -> samples with the residual target
    y = N Kt(T) iq - baseline_loss(w, tau_t, T), tau_t = tau_ref + bearing."""
    out = []
    for w, tau_ref, T, iq in points:
        tau_t = tau_ref + cal.bearing_Nm * math.tanh(w / cfg.bearing_w_eps_rad_s)
        y = cal.gear_ratio * kt_at(cal, T, iq) * iq - baseline_loss(cal, w, tau_t, T, cfg)
        out.append((w, tau_t, T, iq, y))
    return out


class LinearFeatureResidual:
    """Ridge regression on residual_features (the 'PINN surrogate' of the study). Stdlib only."""

    def __init__(self, theta: Optional[Sequence[float]] = None, ridge: float = 1e-6):
        self.theta = list(theta) if theta is not None else None
        self.ridge = ridge

    def fit(self, samples: Iterable[Sample]) -> float:
        n = 16
        A = [[0.0] * n for _ in range(n)]
        b = [0.0] * n
        rows = []
        for w, tau, T, iq, y in samples:
            f = residual_features(w, tau, T, iq)
            rows.append((f, y))
            for i in range(n):
                b[i] += f[i] * y
                fi = f[i]
                Ai = A[i]
                for j in range(n):
                    Ai[j] += fi * f[j]
        for i in range(n):
            A[i][i] += self.ridge * max(1.0, len(rows))
        self.theta = _solve(A, b)
        se = sum((sum(t * x for t, x in zip(self.theta, f)) - y) ** 2 for f, y in rows)
        return math.sqrt(se / max(1, len(rows)))

    def __call__(self, w: float, tau: float, temp_C: float, iq: float) -> float:
        if self.theta is None:
            return 0.0
        return sum(t * x for t, x in zip(self.theta, residual_features(w, tau, temp_C, iq)))

    def to_dict(self) -> dict:
        return {"model": "am_linear_feature_residual_v1", "theta": self.theta, "ridge": self.ridge}

    @classmethod
    def from_dict(cls, d: dict) -> "LinearFeatureResidual":
        return cls(d["theta"], d.get("ridge", 1e-6))


def adoption_check(residual: Residual, samples: Iterable[Sample], min_gain: float = 0.1) -> dict:
    """Compare the loss-model error on held-out dyno samples with and without the residual. Adopt the
    residual only when it cuts the RMS error by at least min_gain (default 10 %)."""
    n, sb, sr = 0, 0.0, 0.0
    for w, tau, T, iq, y in samples:
        sb += y * y
        e = y - residual(w, tau, T, iq)
        sr += e * e
        n += 1
    rb = math.sqrt(sb / max(1, n))
    rr = math.sqrt(sr / max(1, n))
    return {"n": n, "rms_baseline_Nm": rb, "rms_with_residual_Nm": rr, "adopt": n > 0 and rr < (1.0 - min_gain) * rb}
