# SPDX-License-Identifier: Apache-2.0
"""Optional physics-informed neural residual (needs numpy: pip install "am-sdk[pinn]").

SIMULATED, PRE-HARDWARE. Not part of the stdlib core. A small MLP on the physics-structured features
(residual_features) learns the extra loss the baseline model misses. The physics term in the training
loss is the second law for a gearbox at steady speed: the total loss torque cannot feed power into the
joint, (baseline_loss_part + residual) * w >= 0. Adopt it only where adoption_check shows it beats the
baseline (and the linear residual) on dyno data.

Use it as the residual of UkfTorqueEstimator, or call it yourself.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional

try:
    import numpy as np
except ImportError as e:  # pragma: no cover - exercised only without numpy
    raise ImportError('am_sdk.estimation.pinn needs numpy: pip install "am-sdk[pinn]"') from e

from .calibration import EstimatorCalibration
from .ukf import Sample, baseline_loss, residual_features


class PinnResidual:
    def __init__(self, hidden: int = 16, physics_weight: float = 1.0, weight_decay: float = 1e-4, seed: int = 0):
        self.hidden = hidden
        self.physics_weight = physics_weight
        self.weight_decay = weight_decay
        self.seed = seed
        self.params = None
        self.mu = self.sd = None

    def _forward(self, X, p):
        h1 = np.tanh(X @ p["W1"] + p["b1"])
        h2 = np.tanh(h1 @ p["W2"] + p["b2"])
        return (h2 @ p["W3"] + p["b3"]).ravel(), (h1, h2)

    def fit(self, samples: Iterable[Sample], cal: Optional[EstimatorCalibration] = None, epochs: int = 1500,
            lr: float = 3e-3) -> float:
        """Full-batch Adam. cal enables the dissipation penalty (needs the baseline loss)."""
        S = list(samples)
        F = np.array([residual_features(w, t, T, i) for w, t, T, i, _ in S])
        y = np.array([s[4] for s in S])
        w = np.array([s[0] for s in S])
        base = np.array([baseline_loss(cal, s[0], s[1], s[2]) - s[1] for s in S]) if cal is not None else None
        self.mu = F.mean(axis=0)
        self.sd = F.std(axis=0) + 1e-9
        self.sd[14] = 1.0                       # constant feature
        X = (F - self.mu) / self.sd
        rng = np.random.default_rng(self.seed)
        n_in, H = X.shape[1], self.hidden
        p = {"W1": rng.normal(0, 1 / math.sqrt(n_in), (n_in, H)), "b1": np.zeros(H),
             "W2": rng.normal(0, 1 / math.sqrt(H), (H, H)), "b2": np.zeros(H),
             "W3": rng.normal(0, 0.1 / math.sqrt(H), (H, 1)), "b3": np.zeros(1)}
        m = {k: np.zeros_like(v) for k, v in p.items()}
        v = {k: np.zeros_like(v) for k, v in p.items()}
        n = len(y)
        for it in range(1, epochs + 1):
            pred, (h1, h2) = self._forward(X, p)
            g = 2.0 * (pred - y) / n
            if base is not None and self.physics_weight > 0:
                # penalty on (base + pred) * w < 0: relu(-(base + pred) * w)^2
                viol = -(base + pred) * w
                act = viol > 0
                g = g + self.physics_weight * 2.0 * np.where(act, viol * (-w), 0.0) / n
            gW3 = h2.T @ g[:, None]
            gb3 = g.sum(keepdims=True)
            d2 = (g[:, None] @ p["W3"].T) * (1 - h2 ** 2)
            gW2 = h1.T @ d2
            gb2 = d2.sum(axis=0)
            d1 = (d2 @ p["W2"].T) * (1 - h1 ** 2)
            gW1 = X.T @ d1
            gb1 = d1.sum(axis=0)
            grads = {"W1": gW1 + self.weight_decay * p["W1"], "b1": gb1, "W2": gW2 + self.weight_decay * p["W2"],
                     "b2": gb2, "W3": gW3 + self.weight_decay * p["W3"], "b3": gb3}
            for k in p:
                m[k] = 0.9 * m[k] + 0.1 * grads[k]
                v[k] = 0.999 * v[k] + 0.001 * grads[k] ** 2
                p[k] -= lr * (m[k] / (1 - 0.9 ** it)) / (np.sqrt(v[k] / (1 - 0.999 ** it)) + 1e-8)
        self.params = p
        pred, _ = self._forward(X, p)
        return float(np.sqrt(np.mean((pred - y) ** 2)))

    def __call__(self, w: float, tau: float, temp_C: float, iq: float) -> float:
        if self.params is None:
            return 0.0
        X = (np.array([residual_features(w, tau, temp_C, iq)]) - self.mu) / self.sd
        return float(self._forward(X, self.params)[0][0])

    def to_dict(self) -> dict:
        return {"model": "am_pinn_residual_v1", "hidden": self.hidden, "mu": self.mu.tolist(), "sd": self.sd.tolist(),
                "params": {k: v.tolist() for k, v in self.params.items()}}

    @classmethod
    def from_dict(cls, d: dict) -> "PinnResidual":
        r = cls(hidden=d["hidden"])
        r.mu, r.sd = np.array(d["mu"]), np.array(d["sd"])
        r.params = {k: np.array(v) for k, v in d["params"].items()}
        return r
