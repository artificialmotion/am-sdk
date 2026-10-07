# SPDX-License-Identifier: Apache-2.0
"""Runtime actuator model: speed- and temperature-dependent torque limit, losses, thermal network,
peak budget, derate, trip, duty-cycle tracking and events. Pure Python, one actuator per object.

    act = ActuatorModel(load_spec("AM-M"), dt=0.001, ambient_c=25.0, bus_v=48.0)
    for k in range(n):
        tau = act.step(tau_cmd, omega)     # applied output torque after all limits; advances dt
    act.state.winding_c, act.state.derate, act.events, act.duty.rms(10.0)

Sign convention: torque and speed at the output shaft, same sign = motoring.
Everything here is a SIMULATION model: first-order lumped thermal physics calibrated to a datasheet
rating, not a measurement.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional, Tuple

from .spec import ActuatorSpec, DEFAULTS, load_spec, resolve_thermal_nodes

# event kinds
SATURATION_START = "saturation_start"
SATURATION_END = "saturation_end"
PEAK_START = "peak_start"            # torque above the continuous curve (peak budget draining)
PEAK_END = "peak_end"
PEAK_BUDGET_EMPTY = "peak_budget_empty"
PEAK_REARMED = "peak_rearmed"
DERATE_START = "derate_start"
DERATE_END = "derate_end"
TRIP = "trip"
TRIP_CLEAR = "trip_clear"


@dataclass
class Event:
    t: float
    kind: str
    value: float = 0.0     # winding temperature for thermal events, commanded torque for saturation


@dataclass
class ActuatorState:
    t: float = 0.0
    winding_c: float = 25.0
    housing_c: float = 25.0
    ambient_c: float = 25.0
    derate: float = 1.0
    peak_budget_s: float = 0.0
    peak_available: bool = True
    tripped: bool = False
    saturated: bool = False
    above_continuous: bool = False
    limit_pos_nm: float = 0.0
    limit_neg_nm: float = 0.0
    tau_cmd_nm: float = 0.0
    tau_nm: float = 0.0
    omega_rad_s: float = 0.0
    p_copper_w: float = 0.0
    p_mech_w: float = 0.0


class DutyTracker:
    """Running duty-cycle statistics. RMS torque over exponential windows (default 1, 10, 60 s), time
    above the continuous curve, time saturated / derated / tripped, output and loss energy, peak events."""

    def __init__(self, windows_s=(1.0, 10.0, 60.0)):
        self.windows = tuple(float(w) for w in windows_s)
        self.reset()

    def reset(self):
        self.ms = [0.0 for _ in self.windows]
        self.t = 0.0
        self.t_above_continuous = 0.0
        self.t_saturated = 0.0
        self.t_derated = 0.0
        self.t_tripped = 0.0
        self.energy_out_j = 0.0
        self.energy_loss_j = 0.0
        self.peak_events = 0
        self.max_winding_c = -1e9
        self.max_abs_tau_nm = 0.0

    def update(self, dt: float, s: ActuatorState):
        tau2 = s.tau_nm * s.tau_nm
        for i, w in enumerate(self.windows):
            a = dt / w if dt < w else 1.0
            self.ms[i] += a * (tau2 - self.ms[i])
        self.t += dt
        if s.above_continuous:
            self.t_above_continuous += dt
        if s.saturated:
            self.t_saturated += dt
        if s.derate < 1.0:
            self.t_derated += dt
        if s.tripped:
            self.t_tripped += dt
        self.energy_out_j += s.tau_nm * s.omega_rad_s * dt
        self.energy_loss_j += (s.p_copper_w + s.p_mech_w) * dt
        if s.winding_c > self.max_winding_c:
            self.max_winding_c = s.winding_c
        if abs(s.tau_nm) > self.max_abs_tau_nm:
            self.max_abs_tau_nm = abs(s.tau_nm)

    def rms(self, window_s: float = None) -> float:
        i = 0 if window_s is None else min(range(len(self.windows)), key=lambda k: abs(self.windows[k] - window_s))
        return math.sqrt(max(self.ms[i], 0.0))

    def summary(self) -> dict:
        d = {"t_s": self.t, "t_above_continuous_s": self.t_above_continuous, "t_saturated_s": self.t_saturated,
             "t_derated_s": self.t_derated, "t_tripped_s": self.t_tripped, "energy_out_j": self.energy_out_j,
             "energy_loss_j": self.energy_loss_j, "peak_events": self.peak_events,
             "max_winding_c": self.max_winding_c, "max_abs_tau_nm": self.max_abs_tau_nm}
        for w, m in zip(self.windows, self.ms):
            d["rms_%gs_nm" % w] = math.sqrt(max(m, 0.0))
        return d


def friction_torque(spec: ActuatorSpec, omega: float, eps_rad_s: float = 1e-3) -> float:
    """Friction torque at the output (opposes motion; returned with the sign of omega): Coulomb, viscous and
    optional Stribeck, smoothed with tanh(omega/eps). Use it when your simulator does not model joint
    friction itself; MuJoCo's frictionloss/damping (set by the MJCF snippet) is the default instead."""
    fc = spec.coulomb_nm
    if spec.stribeck:
        fs, ws = float(spec.stribeck["static_nm"]), float(spec.stribeck["velocity_rad_s"])
        fc = fc + (fs - fc) * math.exp(-(omega / ws) ** 2)
    return fc * math.tanh(omega / eps_rad_s) + spec.viscous * omega


class ActuatorModel:
    """One actuator. ``step(tau_cmd, omega)`` returns the torque the actuator can deliver.

    Parameters
    ----------
    spec : ActuatorSpec, dict, path or shipped id
    dt : step in seconds (1 kHz = 0.001); step(..., dt=) overrides per call
    ambient_c : ambient temperature (default: the spec's rating ambient)
    bus_v : bus voltage (default: the spec's nominal bus). Curves at other voltages are speed-scaled.
    peak_policy : "budget" (datasheet rule: torque above the continuous curve only while the peak-duration
        budget lasts, then the continuous curve) or "thermal" (peak curve always; only the thermal network
        and derate limit the duration).
    cooling_scale : multiplies the housing-to-ambient (1node: winding-to-ambient) conductance; 1.0 = rating
        conditions. 2.0 halves that resistance (e.g. a better heat sink). An approximation, see the docs.
    initial_c : initial winding/housing temperature (default ambient). Use e.g. 60.0 for a warm start.
    auto_reset_trip : clear the trip once the winding cools by the hysteresis (else call reset_trip()).
    on_event : optional callable(Event)
    """

    def __init__(self, spec, dt: float = 0.001, ambient_c: Optional[float] = None, bus_v: Optional[float] = None,
                 peak_policy: str = "budget", cooling_scale: float = 1.0, initial_c: Optional[float] = None,
                 auto_reset_trip: bool = True, on_event: Optional[Callable[[Event], None]] = None,
                 max_events: int = 10000):
        self.spec = load_spec(spec)
        sp = self.spec
        if peak_policy not in ("budget", "thermal"):
            raise ValueError("peak_policy must be 'budget' or 'thermal'")
        self.dt = float(dt)
        self.peak_policy = peak_policy
        self.auto_reset_trip = auto_reset_trip
        self.on_event = on_event
        self.events: Deque[Event] = deque(maxlen=max_events)
        self.event_counts = {}
        self.duty = DutyTracker()
        cond = sp.conditions
        self.rated_ambient_c = float(cond.get("ambient_c") if cond.get("ambient_c") is not None
                                     else DEFAULTS["ambient_c"][0])
        self.ambient_c = self.rated_ambient_c if ambient_c is None else float(ambient_c)
        self.bus_v = sp.nominal_bus_v if bus_v is None else float(bus_v)
        self._peak_curve, self._peak_scale = sp.curve("peak", self.bus_v)
        self._cont_curve, self._cont_scale = sp.curve("continuous", self.bus_v)
        self._peak_stall = self._peak_curve(0.0)
        self._cont_stall = self._cont_curve(0.0)
        th = sp.thermal
        self.derate_c = float(th["derate_c"])
        self.trip_c = float(th["trip_c"])
        self.trip_hyst = float(th.get("trip_hysteresis_k", DEFAULTS["trip_hysteresis_k"][0]))
        self.derate_law = th.get("derate_law", "sqrt")
        self.alpha = float(th.get("copper_tempco_per_k", DEFAULTS["copper_tempco_per_k"][0]))
        il = th.get("iron_loss") or {}
        self.k_fe1 = float(il.get("k1_w_per_rad_s", 0.0))
        self.k_fe2 = float(il.get("k2_w_s2_per_rad2", 0.0))
        self.mech_heats = bool(th.get("mechanical_losses_heat_actuator", True))
        net = resolve_thermal_nodes(sp)
        self.one_node = net["one_node"]
        self.c_w, self.r_wh, self.c_h, self.r_ha = net["c_w"], net["r_wh"], net["c_h"], net["r_ha"]
        self.cooling_scale = float(cooling_scale)
        # torque cap from the current limit, if published
        self._i_cap_nm = math.inf
        if sp.current_limit_a and sp.kt and sp.ratio:
            self._i_cap_nm = float(sp.current_limit_a) * float(sp.kt) * float(sp.ratio) * sp.gear_efficiency
        self._calibrate_losses()
        self.peak_duration = (float(sp.peak_duration_s) if sp.peak_duration_s else None)
        self.peak_rearm_s = None if self.peak_duration is None else min(0.5, 0.25 * self.peak_duration)
        self.state = ActuatorState()
        self.reset(initial_c)

    # ------------------------------------------------------------------ calibration
    def _mech_loss(self, tau: float, w: float) -> float:
        sp = self.spec
        aw = abs(w)
        p = sp.coulomb_nm * aw + sp.viscous * w * w + self.k_fe1 * aw + self.k_fe2 * w * w
        if sp.gear_efficiency < 1.0:
            p += abs(tau * w) * (1.0 / sp.gear_efficiency - 1.0)
        return p

    def _tau_motor(self, tau: float, w: float) -> float:
        """Output-referred motor torque magnitude: gear loss adds when motoring, subtracts when back-driven."""
        eta = self.spec.gear_efficiency
        return abs(tau) / eta if tau * w >= 0.0 else abs(tau) * eta

    def _calibrate_losses(self):
        sp, th = self.spec, self.spec.thermal
        self.calibration = th.get("loss_calibration", "rating")
        if self.calibration == "electrical":
            r_ref_c = float((self.spec.raw.get("electrical") or {}).get("phase_resistance_ref_c", 25.0))
            kt_out = float(sp.kt) * float(sp.ratio)
            self.k_cu = 1.5 * float(sp.phase_r) / (kt_out * kt_out)
            self.t_ref_c = r_ref_c
            return
        self.t_ref_c = float(th["rated_winding_c"])
        rise = self.t_ref_c - self.rated_ambient_c
        if rise <= 0:
            raise ValueError("%s: rated_winding_c must be above the rating ambient" % sp.id)
        w_r = sp.continuous_speed
        tau_r = sp.continuous_nm
        p_mech = self._mech_loss(tau_r, w_r) if self.mech_heats else 0.0
        if self.one_node:
            p_cu = rise / self.r_wh - p_mech
        else:
            p_cu = (rise - p_mech * self.r_ha) / (self.r_wh + self.r_ha)
        floor = 0.1 * (rise / (self.r_wh + (0.0 if self.one_node else self.r_ha)))
        self.calibration_note = ""
        if p_cu < floor:
            self.calibration_note = ("mechanical losses at the rated point exceed the assumed thermal capacity; "
                                     "copper loss floored at 10 % of the rated heat flow")
            p_cu = floor
        tm = self._tau_motor(tau_r, w_r if w_r > 0 else 1.0)
        self.k_cu = p_cu / (tm * tm)
        self.p_rated_w = p_cu + p_mech

    # ------------------------------------------------------------------ state
    def reset(self, initial_c: Optional[float] = None, ambient_c: Optional[float] = None):
        if ambient_c is not None:
            self.ambient_c = float(ambient_c)
        t0 = self.ambient_c if initial_c is None else float(initial_c)
        s = self.state = ActuatorState(winding_c=t0, housing_c=t0 if not self.one_node else self.ambient_c,
                                       ambient_c=self.ambient_c)
        s.peak_budget_s = self.peak_duration or 0.0
        s.peak_available = True
        self._lockout = False
        s.derate = self._derate_factor(t0)
        self.duty.reset()
        self.events.clear()
        self.event_counts = {}

    def reset_trip(self):
        if self.state.tripped:
            self.state.tripped = False
            self._emit(TRIP_CLEAR, self.state.winding_c)

    def set_ambient(self, ambient_c: float):
        self.ambient_c = self.state.ambient_c = float(ambient_c)

    def _emit(self, kind: str, value: float = 0.0):
        ev = Event(self.state.t, kind, value)
        self.events.append(ev)
        self.event_counts[kind] = self.event_counts.get(kind, 0) + 1
        if self.on_event is not None:
            self.on_event(ev)

    # ------------------------------------------------------------------ limits
    def _derate_factor(self, tw: float) -> float:
        if tw <= self.derate_c:
            return 1.0
        if tw >= self.trip_c:
            return 0.0
        ta = self.ambient_c
        if self.derate_law == "sqrt" and ta < self.derate_c:
            return math.sqrt((self.derate_c - ta) / (tw - ta))
        return max(0.0, 1.0 - (tw - self.derate_c) / (self.trip_c - self.derate_c))

    def curve_limit(self, omega: float, kind: str = "peak") -> float:
        """Published (or built) torque-speed curve value at |omega|, before derate and budget."""
        if kind == "peak":
            return self._peak_curve(abs(omega), self._peak_scale)
        return self._cont_curve(abs(omega), self._cont_scale)

    def torque_limits(self, omega: float) -> Tuple[float, float]:
        """(positive limit, negative-limit magnitude) at output speed omega for the CURRENT state.
        Motoring torque follows the curve; torque against the motion is capped at the stall value."""
        s = self.state
        if s.tripped:
            return 0.0, 0.0
        use_peak = self.peak_policy == "thermal" or s.peak_available
        if use_peak:
            mot = self._peak_curve(abs(omega), self._peak_scale)
            brk = self._peak_stall
        else:
            mot = self._cont_curve(abs(omega), self._cont_scale)
            brk = self._cont_stall
        f = s.derate
        mot = min(mot, self._i_cap_nm)
        brk = min(brk, self._i_cap_nm)
        if f < 1.0:
            # above the derate threshold: no peak; the continuous curve scaled by the derate factor
            mot = min(mot, self._cont_curve(abs(omega), self._cont_scale) * f)
            brk = min(brk, self._cont_stall * f)
        if abs(omega) < 1e-9:
            m = max(mot, brk)
            return m, m
        return (mot, brk) if omega > 0 else (brk, mot)

    # ------------------------------------------------------------------ step
    def losses(self, tau: float, omega: float, winding_c: Optional[float] = None) -> Tuple[float, float]:
        """(copper W, mechanical W) at output torque tau and speed omega."""
        tw = self.state.winding_c if winding_c is None else winding_c
        tm = self._tau_motor(tau, omega)
        rfac = max(0.1, 1.0 + self.alpha * (tw - self.t_ref_c))
        p_cu = self.k_cu * tm * tm * rfac
        p_m = self._mech_loss(tau, omega) if self.mech_heats else 0.0
        return p_cu, p_m

    def step(self, tau_cmd: float, omega: float, dt: Optional[float] = None) -> float:
        dt = self.dt if dt is None else float(dt)
        s = self.state
        pos, neg = self.torque_limits(omega)
        tau = tau_cmd
        if tau > pos:
            tau = pos
        elif tau < -neg:
            tau = -neg
        sat = abs(tau - tau_cmd) > 1e-9 * (1.0 + abs(tau_cmd))
        if sat != s.saturated:
            self._emit(SATURATION_START if sat else SATURATION_END, tau_cmd)
        s.saturated = sat
        s.limit_pos_nm, s.limit_neg_nm = pos, neg
        s.tau_cmd_nm, s.tau_nm, s.omega_rad_s = tau_cmd, tau, omega
        # losses and thermal network (backward Euler, unconditionally stable)
        p_cu, p_m = self.losses(tau, omega)
        s.p_copper_w, s.p_mech_w = p_cu, p_m
        ta = self.ambient_c
        g_scale = self.cooling_scale
        if self.one_node:
            a = self.c_w / dt
            g = g_scale / self.r_wh
            s.winding_c = (a * s.winding_c + p_cu + p_m + g * ta) / (a + g)
            s.housing_c = ta
        else:
            a, b = self.c_w / dt, self.c_h / dt
            g, h = 1.0 / self.r_wh, g_scale / self.r_ha
            rw = a * s.winding_c + p_cu
            rh = b * s.housing_c + p_m + h * ta
            det = (a + g) * (b + g + h) - g * g
            s.winding_c = (rw * (b + g + h) + g * rh) / det
            s.housing_c = ((a + g) * rh + g * rw) / det
        # peak budget
        above = abs(tau) > self.curve_limit(omega, "continuous") * (1.0 + 1e-6) + 1e-9
        if above != s.above_continuous:
            self._emit(PEAK_START if above else PEAK_END, tau)
            if above:
                self.duty.peak_events += 1
        s.above_continuous = above
        if self.peak_duration is not None:
            if above:
                s.peak_budget_s = max(0.0, s.peak_budget_s - dt)
                if s.peak_budget_s <= 1e-9 and not self._lockout:
                    self._lockout = True
                    self._emit(PEAK_BUDGET_EMPTY, s.winding_c)
            else:
                s.peak_budget_s = min(self.peak_duration, s.peak_budget_s + self.spec.peak_recovery * dt)
                if self._lockout and s.peak_budget_s >= self.peak_rearm_s:
                    self._lockout = False
                    self._emit(PEAK_REARMED, s.winding_c)
            s.peak_available = s.peak_budget_s > 1e-9 and not self._lockout
        else:
            s.peak_available = True
        # derate and trip
        old = s.derate
        s.derate = self._derate_factor(s.winding_c)
        if (old < 1.0) != (s.derate < 1.0):
            self._emit(DERATE_START if s.derate < 1.0 else DERATE_END, s.winding_c)
        if not s.tripped and s.winding_c >= self.trip_c:
            s.tripped = True
            self._emit(TRIP, s.winding_c)
        elif s.tripped and self.auto_reset_trip and s.winding_c <= self.trip_c - self.trip_hyst:
            s.tripped = False
            self._emit(TRIP_CLEAR, s.winding_c)
        s.t += dt
        self.duty.update(dt, s)
        return tau

    # ------------------------------------------------------------------ analysis helpers
    def steady_state_winding_c(self, tau: float, omega: float, ambient_c: Optional[float] = None) -> float:
        """Steady-state winding temperature at constant tau and omega (fixed-point on the copper tempco)."""
        ta = self.ambient_c if ambient_c is None else ambient_c
        tw = ta
        for _ in range(100):
            p_cu, p_m = self.losses(tau, omega, tw)
            if self.one_node:
                new = ta + (p_cu + p_m) * self.r_wh / self.cooling_scale
            else:
                new = ta + (p_cu + p_m) * self.r_ha / self.cooling_scale + p_cu * self.r_wh
            if abs(new - tw) < 1e-9:
                break
            tw = new if new < 1e4 else 1e4
        return tw

    def continuous_torque(self, omega: float, ambient_c: Optional[float] = None, limit_c: Optional[float] = None) -> float:
        """Largest constant torque at omega whose steady-state winding stays at or below limit_c (default: the
        derate threshold), also capped by the continuous curve. Use it to check the model against the rating."""
        lim = self.derate_c if limit_c is None else limit_c
        hi = self.curve_limit(omega, "peak")
        lo = 0.0
        if self.steady_state_winding_c(0.0, omega, ambient_c) > lim:
            return 0.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self.steady_state_winding_c(mid, omega, ambient_c) <= lim:
                lo = mid
            else:
                hi = mid
        return lo

    def friction(self, omega: float) -> float:
        return friction_torque(self.spec, omega)

    def summary(self) -> dict:
        s = self.state
        d = {"id": self.spec.id, "t_s": s.t, "winding_c": s.winding_c, "housing_c": s.housing_c, "derate": s.derate,
             "peak_budget_s": s.peak_budget_s, "tripped": s.tripped}
        d.update(self.duty.summary())
        d["events"] = dict(self.event_counts)
        return d
