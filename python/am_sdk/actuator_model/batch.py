"""Vectorised actuator model for many environments x joints (RL training). Works on numpy arrays or
torch tensors (CPU or GPU) with the same code; the scalar ActuatorModel is the reference and the tests
hold the two together.

    bank = BatchActuatorModel([load_spec("AM-M")] * 2 + [load_spec("AM-H-T")], num_envs=4096, dt=0.005,
                              backend="torch", device="cuda")
    tau = bank.step(tau_cmd, omega)          # (num_envs, num_joints) in and out

Difference from the scalar model: events are counted per env/joint (``counts``) instead of being
logged. Curves are interpolated exactly (padded breakpoint tables + searchsorted).
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

from .model import ActuatorModel
from .spec import load_spec


class _Ops:
    def __init__(self, backend: str, device=None, dtype=None):
        self.backend = backend
        if backend == "numpy":
            import numpy as np
            self.xp = np
            self.dtype = dtype or np.float64
            self.device = None
        elif backend == "torch":
            import torch
            self.xp = torch
            self.dtype = dtype or torch.float32
            self.device = device or "cpu"
        else:
            raise ValueError("backend must be 'numpy' or 'torch'")

    def asarray(self, x, dtype=None):
        dt = self.dtype if dtype is None else dtype
        if self.backend == "numpy":
            return self.xp.asarray(x, dtype=dt)
        if isinstance(x, self.xp.Tensor):
            return x.to(device=self.device, dtype=dt)
        return self.xp.as_tensor(x, dtype=dt, device=self.device)

    def full(self, shape, v, dtype=None):
        if self.backend == "numpy":
            return self.xp.full(shape, v, dtype=self.dtype if dtype is None else dtype)
        return self.xp.full(shape, v, dtype=self.dtype if dtype is None else dtype, device=self.device)

    def bool_full(self, shape, v: bool):
        if self.backend == "numpy":
            return self.xp.full(shape, v, dtype=bool)
        return self.xp.full(shape, v, dtype=self.xp.bool, device=self.device)

    def arange(self, n):
        if self.backend == "numpy":
            return self.xp.arange(n)
        return self.xp.arange(n, device=self.device)

    def to_index(self, x):
        if self.backend == "numpy":
            return x.astype(self.xp.int64)
        return x.long()

    def clip(self, x, lo, hi):
        if self.backend == "numpy":
            return self.xp.clip(x, lo, hi)
        return self.xp.clamp(x, lo, hi)


class BatchActuatorModel:
    """Batched counterpart of ActuatorModel. ``specs`` is one spec (or id/path) per joint column."""

    def __init__(self, specs: Sequence, num_envs: int, dt: float, backend: str = "numpy", device=None, dtype=None,
                 ambient_c: Optional[float] = None, bus_v: Optional[float] = None, peak_policy: str = "budget",
                 cooling_scale: float = 1.0, initial_c: Optional[float] = None,
                 rms_window_s: float = 10.0, auto_reset_trip: bool = True):
        self.ops = o = _Ops(backend, device, dtype)
        xp = o.xp
        self.specs = [load_spec(s) for s in specs]
        self.ref = [ActuatorModel(s, dt=dt, ambient_c=ambient_c, bus_v=bus_v, peak_policy=peak_policy,
                                  cooling_scale=cooling_scale) for s in self.specs]
        self.N, self.M, self.dt = int(num_envs), len(self.specs), float(dt)
        self.peak_policy = peak_policy
        self.auto_reset_trip = auto_reset_trip
        self.rms_window_s = float(rms_window_s)
        refs = self.ref

        def vec(vals):
            return o.asarray([float(v) for v in vals])

        def table(kind):
            ws, ts, ends = [], [], []
            for r in refs:
                c, sc = (r._peak_curve, r._peak_scale) if kind == "peak" else (r._cont_curve, r._cont_scale)
                ws.append([w * sc for w in c.w])
                ts.append(list(c.t))
                ends.append(c.w_max * sc)
            P = max(len(w) for w in ws)
            for w, t in zip(ws, ts):          # pad with points beyond the end (never used: torque is 0 there)
                while len(w) < P:
                    w.append(w[-1] + 1.0)
                    t.append(t[-1])
            return o.asarray(ws), o.asarray(ts), vec(ends)

        self.pk_w, self.pk_t, self.pk_end = table("peak")
        self.ct_w, self.ct_t, self.ct_end = table("continuous")
        self.pk_stall = vec(r._peak_stall for r in refs)
        self.ct_stall = vec(r._cont_stall for r in refs)
        self.i_cap = vec(min(r._i_cap_nm, 1e12) for r in refs)
        self.eta = vec(r.spec.gear_efficiency for r in refs)
        self.coulomb = vec(r.spec.coulomb_nm for r in refs)
        self.viscous = vec(r.spec.viscous for r in refs)
        self.k_fe1 = vec(r.k_fe1 for r in refs)
        self.k_fe2 = vec(r.k_fe2 for r in refs)
        self.mech_on = vec(1.0 if r.mech_heats else 0.0 for r in refs)
        self.k_cu = vec(r.k_cu for r in refs)
        self.alpha = vec(r.alpha for r in refs)
        self.t_ref = vec(r.t_ref_c for r in refs)
        self.one = o.asarray([r.one_node for r in refs], dtype=(bool if backend == "numpy" else xp.bool))
        self.c_w = vec(r.c_w for r in refs)
        self.r_wh = vec(r.r_wh for r in refs)
        self.c_h = vec((r.c_h if not r.one_node else 1.0) for r in refs)
        self.r_ha = vec((r.r_ha if not r.one_node else 1.0) for r in refs)
        self.derate_c = vec(r.derate_c for r in refs)
        self.trip_c = vec(r.trip_c for r in refs)
        self.hyst = vec(r.trip_hyst for r in refs)
        self.sqrt_law = o.asarray([r.derate_law == "sqrt" for r in refs], dtype=(bool if backend == "numpy" else xp.bool))
        self.has_budget = o.asarray([r.peak_duration is not None for r in refs],
                                    dtype=(bool if backend == "numpy" else xp.bool))
        self.peak_dur = vec((r.peak_duration or 1.0) for r in refs)
        self.recovery = vec(r.spec.peak_recovery for r in refs)
        self.rearm = vec((r.peak_rearm_s or 0.0) for r in refs)
        self.cooling_scale = float(cooling_scale)
        self.ambient_c = float(refs[0].ambient_c if ambient_c is None else ambient_c)
        self._jidx = o.arange(self.M)[None, :]
        self.reset(initial_c=initial_c)

    # ------------------------------------------------------------------ state
    def reset(self, env_ids=None, initial_c: Optional[float] = None):
        o = self.ops
        t0 = self.ambient_c if initial_c is None else float(initial_c)
        shape = (self.N, self.M)
        if env_ids is None or not hasattr(self, "tw"):
            self.tw = o.full(shape, t0)
            self.th = o.full(shape, t0)
            self.th = o.xp.where(self.one[None, :], o.full(shape, self.ambient_c), self.th)
            self.budget = o.full(shape, 0.0) + self.peak_dur[None, :]
            self.lockout = o.bool_full(shape, False)
            self.tripped = o.bool_full(shape, False)
            self.saturated = o.bool_full(shape, False)
            self.above = o.bool_full(shape, False)
            self.derate = o.full(shape, 1.0)
            self.tau2_ms = o.full(shape, 0.0)
            self.counts = {k: o.full(shape, 0.0) for k in ("saturation_steps", "derate_start", "trip", "peak_budget_empty",
                                                         "peak_start")}
            self.limit_pos = o.full(shape, 0.0)
            self.limit_neg = o.full(shape, 0.0)
            self.p_cu = o.full(shape, 0.0)
            self.p_mech = o.full(shape, 0.0)
            return
        ids = env_ids
        self.tw[ids] = t0
        self.th[ids] = t0
        self.th[ids] = o.xp.where(self.one[None, :], o.full((1, self.M), self.ambient_c), self.th[ids])
        self.budget[ids] = self.peak_dur[None, :]
        for a in (self.lockout, self.tripped, self.saturated, self.above):
            a[ids] = False
        self.derate[ids] = 1.0
        self.tau2_ms[ids] = 0.0
        for v in self.counts.values():
            v[ids] = 0.0

    # ------------------------------------------------------------------ curves
    def _curve(self, kind, aw):
        """Exact piecewise-linear interpolation of each joint's curve at |omega| (same as Curve.__call__)."""
        o, xp = self.ops, self.ops.xp
        W, T, end = (self.pk_w, self.pk_t, self.pk_end) if kind == "peak" else (self.ct_w, self.ct_t, self.ct_end)
        P = W.shape[1]
        if o.backend == "torch":
            idx = xp.searchsorted(W, aw.transpose(0, 1).contiguous(), right=True).transpose(0, 1)
        else:
            idx = xp.empty(aw.shape, dtype=xp.int64)
            for j in range(self.M):
                idx[:, j] = xp.searchsorted(W[j], aw[:, j], side="right")
        i1 = o.clip(idx, 1, P - 1)
        i0 = i1 - 1
        w0, w1 = W[self._jidx, i0], W[self._jidx, i1]
        t0, t1 = T[self._jidx, i0], T[self._jidx, i1]
        span = w1 - w0
        frac = o.clip((aw - w0) / xp.where(span > 0, span, xp.ones_like(span)), 0.0, 1.0)
        v = xp.where(span > 0, t0 + (t1 - t0) * frac, xp.minimum(t0, t1))
        v = xp.where(aw <= W[:, 0][None, :], T[:, 0][None, :] + 0.0 * aw, v)
        return xp.where(aw > end[None, :] + 1e-9, xp.zeros_like(v), v)

    def _derate_factor(self, tw):
        o, xp = self.ops, self.ops.xp
        dc, tc = self.derate_c[None, :], self.trip_c[None, :]
        ta = self.ambient_c
        lin = o.clip(1.0 - (tw - dc) / (tc - dc), 0.0, 1.0)
        sq = xp.sqrt(xp.maximum(dc - ta, 0.0 * dc) / xp.maximum(tw - ta, 1e-9 + 0.0 * tw))
        use_sq = self.sqrt_law[None, :] & (dc > ta)
        f = xp.where(use_sq, o.clip(sq, 0.0, 1.0), lin)
        f = xp.where(tw <= dc, xp.ones_like(f), f)
        return xp.where(tw >= tc, xp.zeros_like(f), f)

    def torque_limits(self, omega):
        o, xp = self.ops, self.ops.xp
        aw = xp.abs(omega)
        use_peak = (self.budget > 1e-9) & ~self.lockout | ~self.has_budget[None, :]
        if self.peak_policy == "thermal":
            use_peak = xp.ones_like(use_peak)
        mot = xp.where(use_peak, self._curve("peak", aw), self._curve("continuous", aw))
        brk = xp.where(use_peak, self.pk_stall[None, :] + 0.0 * aw, self.ct_stall[None, :] + 0.0 * aw)
        cap = self.i_cap[None, :]
        f = self.derate
        hot = f < 1.0
        mot = xp.minimum(mot, cap)
        brk = xp.minimum(brk, cap)
        mot = xp.where(hot, xp.minimum(mot, self._curve("continuous", aw) * f), mot)
        brk = xp.where(hot, xp.minimum(brk, self.ct_stall[None, :] * f), brk)
        still = aw < 1e-9
        m = xp.maximum(mot, brk)
        pos = xp.where(still, m, xp.where(omega > 0, mot, brk))
        neg = xp.where(still, m, xp.where(omega > 0, brk, mot))
        pos = xp.where(self.tripped, xp.zeros_like(pos), pos)
        neg = xp.where(self.tripped, xp.zeros_like(neg), neg)
        return pos, neg

    # ------------------------------------------------------------------ step
    def step(self, tau_cmd, omega, dt: Optional[float] = None):
        o, xp = self.ops, self.ops.xp
        dt = self.dt if dt is None else float(dt)
        tau_cmd = o.asarray(tau_cmd)
        omega = o.asarray(omega)
        pos, neg = self.torque_limits(omega)
        tau = xp.minimum(xp.maximum(tau_cmd, -neg), pos)
        sat = xp.abs(tau - tau_cmd) > 1e-9 * (1.0 + xp.abs(tau_cmd))
        self.saturated = sat
        self.counts["saturation_steps"] += sat * 1.0
        self.limit_pos, self.limit_neg = pos, neg
        # losses
        aw = xp.abs(omega)
        eta = self.eta[None, :]
        tm = xp.where(tau * omega >= 0.0, xp.abs(tau) / eta, xp.abs(tau) * eta)
        rfac = xp.maximum(1.0 + self.alpha[None, :] * (self.tw - self.t_ref[None, :]), 0.1 + 0.0 * tm)
        p_cu = self.k_cu[None, :] * tm * tm * rfac
        p_m = (self.coulomb[None, :] * aw + self.viscous[None, :] * omega * omega + self.k_fe1[None, :] * aw
               + self.k_fe2[None, :] * omega * omega + xp.abs(tau * omega) * (1.0 / eta - 1.0)) * self.mech_on[None, :]
        self.p_cu, self.p_mech = p_cu, p_m
        ta = self.ambient_c
        a = self.c_w[None, :] / dt
        g1 = self.cooling_scale / self.r_wh[None, :]
        tw1 = (a * self.tw + p_cu + p_m + g1 * ta) / (a + g1)
        b = self.c_h[None, :] / dt
        g = 1.0 / self.r_wh[None, :]
        h = self.cooling_scale / self.r_ha[None, :]
        rw = a * self.tw + p_cu
        rh = b * self.th + p_m + h * ta
        det = (a + g) * (b + g + h) - g * g
        tw2 = (rw * (b + g + h) + g * rh) / det
        th2 = ((a + g) * rh + g * rw) / det
        one = self.one[None, :]
        self.tw = xp.where(one, tw1, tw2)
        self.th = xp.where(one, ta + 0.0 * tw1, th2)
        # peak budget
        cont = self._curve("continuous", aw)
        above = xp.abs(tau) > cont * (1.0 + 1e-6) + 1e-9
        self.counts["peak_start"] += (above & ~self.above) * 1.0
        self.above = above
        hb = self.has_budget[None, :]
        drained = xp.maximum(self.budget - dt, 0.0 * self.budget)
        refill = xp.minimum(self.budget + self.recovery[None, :] * dt, self.peak_dur[None, :] + 0.0 * self.budget)
        new_budget = xp.where(above, drained, refill)
        newly_empty = hb & above & (new_budget <= 1e-9) & ~self.lockout
        self.counts["peak_budget_empty"] += newly_empty * 1.0
        lock = self.lockout | newly_empty
        lock = lock & ~(~above & (new_budget >= self.rearm[None, :]))
        self.lockout = lock & hb
        self.budget = xp.where(hb, new_budget, self.budget)
        # derate / trip
        old = self.derate
        self.derate = self._derate_factor(self.tw)
        self.counts["derate_start"] += ((self.derate < 1.0) & (old >= 1.0)) * 1.0
        trip_now = ~self.tripped & (self.tw >= self.trip_c[None, :])
        self.counts["trip"] += trip_now * 1.0
        tripped = self.tripped | trip_now
        if self.auto_reset_trip:
            tripped = tripped & ~(self.tw <= (self.trip_c - self.hyst)[None, :])
        self.tripped = tripped
        a_rms = min(dt / self.rms_window_s, 1.0)
        self.tau2_ms = self.tau2_ms + a_rms * (tau * tau - self.tau2_ms)
        return tau

    @property
    def rms_torque(self):
        return self.ops.xp.sqrt(self.tau2_ms)
