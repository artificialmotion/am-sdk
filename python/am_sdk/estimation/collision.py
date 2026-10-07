"""Momentum-observer collision detector (non-safety).

SIMULATED, PRE-HARDWARE. This is a host-side detector for reacting to unexpected contact. It is not
a safety function and does not replace STO or a certified safety controller.

For one joint with total inertia J (armature + your link model) the generalised momentum p = J qd obeys

    dp/dt = tau_drive + tau_known + tau_contact

tau_drive is the estimator's drive torque (current model after friction and gear losses, without the
armature inertia term), tau_known is what your model of the link explains (gravity, coupling). The
observer residual

    r_k = K * (p_k - p_0 - sum_j (tau_drive_j + tau_known_j + r_{j-1}) * dt)

follows tau_contact through a first-order lag of bandwidth K (rad/s), without differentiating the
velocity. A collision is flagged when |r| exceeds the threshold; the threshold is raised while the
estimator reports STICK or AMBIGUOUS (friction uncertain), and the flag clears with hysteresis.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional

from .core import EST_AMBIGUOUS, EST_STICK

TWO_PI = 2.0 * math.pi


@dataclass
class CollisionConfig:
    inertia_kgm2: float               # armature + link inertia seen by the joint
    gain_hz: float = 30.0             # observer bandwidth
    threshold_Nm: float = math.inf    # inf = report the residual only (calibrate with calibrate_threshold)
    stick_scale: float = 2.0          # threshold multiplier while the estimate is STICK or AMBIGUOUS
    release_ratio: float = 0.5        # clear below release_ratio * threshold ...
    min_hold_s: float = 0.05          # ... held for this long


@dataclass
class CollisionOutput:
    residual_Nm: float = 0.0          # estimated unexpected (contact) torque, observer-filtered
    threshold_Nm: float = math.inf    # threshold in force for this sample
    active: bool = False
    onset: bool = False               # True on the first sample of a detection
    t_s: float = 0.0


@dataclass
class CollisionEvent:
    t_s: float
    residual_Nm: float


class CollisionDetector:
    def __init__(self, cfg: CollisionConfig):
        self.cfg = cfg
        self.events: List[CollisionEvent] = []
        self.reset()

    def reset(self, qd_rad_s: Optional[float] = None):
        self._init = qd_rad_s is not None
        self.p0 = self.cfg.inertia_kgm2 * (qd_rad_s or 0.0)
        self.integ = 0.0
        self.r = 0.0
        self.active = False
        self.hold = 0.0
        self.t = 0.0

    def update(self, dt: float, qd_rad_s: float, tau_drive_Nm: float, tau_known_Nm: float = 0.0,
               quality: int = 0) -> CollisionOutput:
        c = self.cfg
        if not self._init:
            self.p0 = c.inertia_kgm2 * qd_rad_s
            self._init = True
        self.t += dt
        self.integ += (tau_drive_Nm + tau_known_Nm + self.r) * dt
        self.r = TWO_PI * c.gain_hz * (c.inertia_kgm2 * qd_rad_s - self.p0 - self.integ)
        th = c.threshold_Nm * (c.stick_scale if quality & (EST_STICK | EST_AMBIGUOUS) else 1.0)
        onset = False
        if not self.active:
            if abs(self.r) > th:
                self.active, onset, self.hold = True, True, 0.0
                self.events.append(CollisionEvent(self.t, self.r))
        else:
            if abs(self.r) < c.release_ratio * th:
                self.hold += dt
                if self.hold >= c.min_hold_s:
                    self.active = False
            else:
                self.hold = 0.0
        return CollisionOutput(residual_Nm=self.r, threshold_Nm=th, active=self.active, onset=onset, t_s=self.t)


def calibrate_threshold(residuals: Iterable[float], margin: float = 1.1) -> float:
    """Threshold from a contact-free run of the motions you will use: margin x max |residual|.
    Run it per robot and per motion set; it is not a published number."""
    m = 0.0
    for r in residuals:
        if math.isfinite(r):
            m = max(m, abs(r))
    return margin * m
