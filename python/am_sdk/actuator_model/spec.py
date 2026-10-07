# SPDX-License-Identifier: Apache-2.0
"""Actuator spec: a JSON description any vendor's datasheet fits.

A spec holds what a datasheet states (ratio, torque-speed curves, ratings and their conditions,
inertia, friction, efficiency, optional electrical data) plus a thermal network. Every value that
is not from the source is listed in ``assumptions`` with its basis, so a reader can see what the
model invented.

    spec = load_spec("AM-M")                       # shipped spec, by id
    spec = load_spec("path/to/my_actuator.json")   # your own file
    spec = spec_from_datasheet(id="my-unit", rated_nm=20, peak_nm=50, mass_kg=0.8, ratio=10,
                               rated_speed_rpm=150)  # minimal data, documented defaults
    errors = validate_spec(spec.to_dict())

All torques, speeds and inertias are at the OUTPUT shaft. Speeds are stored in rad/s after loading
(``speed_unit`` may be "rpm" or "rad/s" in the file).
"""
from __future__ import annotations

import copy
import json
import math
import os
from typing import Dict, List, Optional, Tuple

SCHEMA_ID = "am-actuator-spec/1"
RPM = 2.0 * math.pi / 60.0
SPEC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "specs")

# Default assumptions used by spec_from_datasheet() and by load-time normalisation.
# Each one is written into spec["assumptions"] when it is used.
DEFAULTS = {
    "ambient_c": (25.0, "rating ambient not stated; 25 C assumed (common catalogue reference)"),
    "rated_winding_c": (105.0, "winding temperature at the continuous rating not stated; the continuous rating is taken as "
                               "the steady state that brings the winding to the derate threshold (105 C)"),
    "derate_c": (105.0, "derate threshold not stated; 105 C assumed (class F/H wire with margin)"),
    "trip_c": (130.0, "trip temperature not stated; 130 C assumed"),
    "trip_hysteresis_k": (5.0, "trip reset hysteresis not stated; 5 K assumed"),
    "c_specific_j_per_kg_k": (600.0, "lumped heat capacity = 600 J/(kg K) x actuator mass (aluminium/steel/copper/magnet "
                                     "mix); an order-of-magnitude assumption"),
    "tau_housing_s": (300.0, "thermal time constant not stated; 300 s assumed for the housing (dominant) mode"),
    "tau_winding_s": (30.0, "winding-to-housing time constant not stated; 30 s assumed"),
    "winding_rise_fraction": (0.5, "share of the rated temperature rise across winding-to-housing not stated; 0.5 assumed"),
    "gear_efficiency": (0.9, "gear efficiency not stated; 0.90 assumed (single-stage planetary order of magnitude)"),
    "coulomb_fraction_of_peak": (0.02, "friction / backdrive torque not stated; 2 % of peak torque assumed"),
    "viscous_nm_s_per_rad": (0.0, "viscous friction not stated; 0 assumed"),
    "no_load_over_rated_speed": (1.25, "no-load speed not stated; 1.25 x rated speed assumed"),
    "inertia_per_peak_nm": (4.0e-4, "reflected inertia not stated; 4e-4 kg m^2 per N m of peak torque assumed "
                                    "(order of magnitude for geared quasi-direct-drive units)"),
    "peak_duration_s": (2.0, "peak duration not stated; 2 s assumed"),
    "peak_recovery_s_per_s": (0.1, "peak recovery not stated; 0.1 s of peak per s below continuous assumed"),
    "copper_tempco_per_k": (0.00393, "copper resistance temperature coefficient (physical constant)"),
}

COOLING_TYPES = ("bolted_structure", "plate", "still_air", "forced_air", "liquid", "unstated")


class SpecError(ValueError):
    pass


# --------------------------------------------------------------------------------------------- helpers

def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _get(d: dict, path: str, default=None):
    cur = d
    for p in path.split("."):
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


def _assume(spec: dict, field: str, value, basis: str):
    lst = spec.setdefault("assumptions", [])
    for a in lst:
        if a.get("field") == field:
            return
    lst.append({"field": field, "value": value, "basis": basis})


# --------------------------------------------------------------------------------------------- validation

def validate_spec(d: dict) -> List[str]:
    """Return a list of problems (empty when the spec is usable). Stdlib only; mirrors actuator_spec.schema.json."""
    e: List[str] = []
    if not isinstance(d, dict):
        return ["spec must be a JSON object"]
    if d.get("schema") != SCHEMA_ID:
        e.append("schema must be %r" % SCHEMA_ID)
    if not isinstance(d.get("id"), str) or not d.get("id"):
        e.append("id must be a non-empty string")
    mech = d.get("mechanical")
    if not isinstance(mech, dict):
        e.append("mechanical: object required")
        mech = {}
    for k in ("mass_kg", "reflected_inertia_kgm2"):
        if not (_num(mech.get(k)) and mech.get(k) > 0):
            e.append("mechanical.%s must be a positive number" % k)
    if mech.get("ratio") is not None and not (_num(mech.get("ratio")) and mech["ratio"] > 0):
        e.append("mechanical.ratio must be positive or null")
    ge = mech.get("gear_efficiency")
    if ge is not None and not (_num(ge) and 0 < ge <= 1):
        e.append("mechanical.gear_efficiency must be in (0, 1] or null")
    r = d.get("ratings")
    if not isinstance(r, dict):
        e.append("ratings: object required")
        r = {}
    for k in ("peak_nm", "continuous_nm"):
        if not (_num(r.get(k)) and r.get(k) > 0):
            e.append("ratings.%s must be a positive number" % k)
    if _num(r.get("peak_nm")) and _num(r.get("continuous_nm")) and r["continuous_nm"] > r["peak_nm"] + 1e-9:
        e.append("ratings.continuous_nm exceeds ratings.peak_nm")
    if not (_num(r.get("nominal_bus_v")) and r["nominal_bus_v"] > 0):
        e.append("ratings.nominal_bus_v must be a positive number")
    cond = r.get("conditions")
    if not isinstance(cond, dict) or not isinstance(cond.get("stated"), bool):
        e.append("ratings.conditions.stated (bool) required")
    elif cond.get("cooling") is not None and cond.get("cooling") not in COOLING_TYPES:
        e.append("ratings.conditions.cooling must be one of %s" % ", ".join(COOLING_TYPES))
    ts = d.get("torque_speed", [])
    if not isinstance(ts, list):
        e.append("torque_speed must be a list")
        ts = []
    for i, c in enumerate(ts):
        if not isinstance(c, dict):
            e.append("torque_speed[%d] must be an object" % i)
            continue
        if c.get("kind") not in ("peak", "continuous"):
            e.append("torque_speed[%d].kind must be 'peak' or 'continuous'" % i)
        if not (_num(c.get("bus_v")) and c["bus_v"] > 0):
            e.append("torque_speed[%d].bus_v must be positive" % i)
        if c.get("speed_unit", "rad/s") not in ("rpm", "rad/s"):
            e.append("torque_speed[%d].speed_unit must be 'rpm' or 'rad/s'" % i)
        pts = c.get("points")
        if not isinstance(pts, list) or len(pts) < 2:
            e.append("torque_speed[%d].points needs at least 2 [speed, torque] pairs" % i)
            continue
        prev = -1.0
        for p in pts:
            if not (isinstance(p, (list, tuple)) and len(p) == 2 and _num(p[0]) and _num(p[1]) and p[1] >= 0):
                e.append("torque_speed[%d].points: bad point %r" % (i, p))
                break
            if p[0] < prev:
                e.append("torque_speed[%d].points: speeds must be non-decreasing" % i)
                break
            prev = p[0]
    fr = d.get("friction", {})
    if not isinstance(fr, dict):
        e.append("friction must be an object")
    else:
        for k in ("coulomb_nm", "viscous_nm_s_per_rad"):
            if fr.get(k) is not None and not (_num(fr[k]) and fr[k] >= 0):
                e.append("friction.%s must be >= 0" % k)
        st = fr.get("stribeck")
        if st is not None and not (isinstance(st, dict) and _num(st.get("static_nm")) and _num(st.get("velocity_rad_s"))
                                   and st["velocity_rad_s"] > 0):
            e.append("friction.stribeck needs static_nm and velocity_rad_s > 0")
    th = d.get("thermal")
    if not isinstance(th, dict):
        e.append("thermal: object required")
        return e
    if th.get("model") not in ("1node", "2node"):
        e.append("thermal.model must be '1node' or '2node'")
    if th.get("loss_calibration", "rating") not in ("rating", "electrical"):
        e.append("thermal.loss_calibration must be 'rating' or 'electrical'")
    for k in ("derate_c", "trip_c"):
        if not _num(th.get(k)):
            e.append("thermal.%s must be a number" % k)
    if _num(th.get("derate_c")) and _num(th.get("trip_c")) and th["trip_c"] <= th["derate_c"]:
        e.append("thermal.trip_c must be above thermal.derate_c")
    if th.get("derate_law", "sqrt") not in ("sqrt", "linear"):
        e.append("thermal.derate_law must be 'sqrt' or 'linear'")
    n = th.get("nodes")
    if not isinstance(n, dict):
        e.append("thermal.nodes: object required")
    elif th.get("model") == "1node":
        if not (("tau_s" in n and "c_j_per_k" in n) or ("r_k_per_w" in n and "c_j_per_k" in n)):
            e.append("thermal.nodes (1node) needs c_j_per_k and tau_s or r_k_per_w")
    elif th.get("model") == "2node":
        rc = all(k in n for k in ("c_winding_j_per_k", "r_winding_housing_k_per_w", "c_housing_j_per_k",
                                  "r_housing_ambient_k_per_w"))
        tc = all(k in n for k in ("tau_housing_s", "tau_winding_s", "winding_rise_fraction", "c_total_j_per_k"))
        if not (rc or tc):
            e.append("thermal.nodes (2node) needs the four R/C values or tau_housing_s, tau_winding_s, "
                     "winding_rise_fraction, c_total_j_per_k")
    if th.get("loss_calibration") == "electrical":
        el = d.get("electrical") or {}
        if not (_num(el.get("kt_nm_per_a")) and _num(el.get("phase_resistance_ohm")) and _num(mech.get("ratio"))):
            e.append("loss_calibration 'electrical' needs electrical.kt_nm_per_a, electrical.phase_resistance_ohm "
                     "and mechanical.ratio")
    elif not _num(th.get("rated_winding_c")):
        e.append("thermal.rated_winding_c must be a number for loss_calibration 'rating'")
    if not isinstance(d.get("assumptions", []), list):
        e.append("assumptions must be a list")
    return e


# --------------------------------------------------------------------------------------------- spec object

class Curve:
    """Piecewise-linear torque-speed curve at the output, speed in rad/s (absolute value used)."""

    __slots__ = ("w", "t", "bus_v", "kind")

    def __init__(self, points: List[Tuple[float, float]], bus_v: float, kind: str):
        self.w = [float(p[0]) for p in points]
        self.t = [float(p[1]) for p in points]
        self.bus_v = float(bus_v)
        self.kind = kind

    @property
    def w_max(self) -> float:
        return self.w[-1]

    def __call__(self, w_abs: float, speed_scale: float = 1.0) -> float:
        """Torque at |speed| w_abs; speed_scale stretches the speed axis (bus voltage scaling). 0 beyond the end."""
        ws, ts = self.w, self.t
        x = w_abs / speed_scale
        if x <= ws[0]:
            return ts[0]
        if x >= ws[-1]:
            return ts[-1] if x <= ws[-1] + 1e-9 else 0.0
        lo, hi = 0, len(ws) - 1
        while hi - lo > 1:
            mid = (lo + hi) >> 1
            if ws[mid] <= x:
                lo = mid
            else:
                hi = mid
        w0, w1 = ws[lo], ws[hi]
        if w1 <= w0:
            return min(ts[lo], ts[hi])
        return ts[lo] + (ts[hi] - ts[lo]) * (x - w0) / (w1 - w0)


class ActuatorSpec:
    """A validated, normalised spec. ``raw`` is the JSON dict (with units as written); the attributes are SI."""

    def __init__(self, d: dict):
        d = copy.deepcopy(d)
        errs = validate_spec(d)
        if errs:
            raise SpecError("invalid actuator spec %r:\n  - %s" % (d.get("id"), "\n  - ".join(errs)))
        self.raw = d
        self.id = d["id"]
        self.vendor = d.get("vendor", "")
        self.model_name = d.get("model", "")
        self.status = d.get("status", "")
        m, r, th = d["mechanical"], d["ratings"], d["thermal"]
        self.ratio = m.get("ratio")
        self.mass_kg = float(m["mass_kg"])
        self.inertia_kgm2 = float(m["reflected_inertia_kgm2"])
        self.gear_efficiency = float(m["gear_efficiency"]) if m.get("gear_efficiency") is not None else 1.0
        self.backdrive_nm = m.get("backdrive_nm")
        self.peak_nm = float(r["peak_nm"])
        self.continuous_nm = float(r["continuous_nm"])
        self.continuous_speed = (float(r["continuous_speed_rad_s"]) if r.get("continuous_speed_rad_s") is not None
                                 else 0.0)
        self.peak_duration_s = r.get("peak_duration_s")
        self.peak_recovery = float(r.get("peak_recovery_s_per_s") or DEFAULTS["peak_recovery_s_per_s"][0])
        self.nominal_bus_v = float(r["nominal_bus_v"])
        self.no_load = {float(k): float(v) for k, v in (r.get("no_load_speed_rad_s") or {}).items()}
        self.conditions = r["conditions"]
        self.curves: List[Curve] = []
        for c in d.get("torque_speed", []):
            k = RPM if c.get("speed_unit", "rad/s") == "rpm" else 1.0
            self.curves.append(Curve([(p[0] * k, p[1]) for p in c["points"]], c["bus_v"], c["kind"]))
        if not any(c.kind == "peak" for c in self.curves):
            raise SpecError("%s: no peak torque-speed curve; use spec_from_datasheet() to build one" % self.id)
        fr = d.get("friction", {})
        self.coulomb_nm = float(fr.get("coulomb_nm") or 0.0)
        self.viscous = float(fr.get("viscous_nm_s_per_rad") or 0.0)
        self.stribeck = fr.get("stribeck")
        el = d.get("electrical") or {}
        self.kt = el.get("kt_nm_per_a")
        self.phase_r = el.get("phase_resistance_ohm")
        self.current_limit_a = el.get("current_limit_a")
        self.thermal = th
        self.assumptions = d.get("assumptions", [])

    # ---- curves
    def curve(self, kind: str, bus_v: Optional[float] = None) -> Tuple[Curve, float]:
        """(curve, speed_scale) for the requested bus. Exact bus match if published; otherwise the nearest bus
        with its speed axis scaled by bus_v / curve_bus_v (back-EMF scaling, an approximation)."""
        bus_v = self.nominal_bus_v if bus_v is None else float(bus_v)
        cands = [c for c in self.curves if c.kind == kind]
        if not cands:
            # no continuous curve: rated torque flat to the end of the peak curve
            pk, sc = self.curve("peak", bus_v)
            flat = Curve([(0.0, min(self.continuous_nm, pk.t[0])), (pk.w_max, min(self.continuous_nm, pk.t[-1]))],
                         pk.bus_v, kind)
            return flat, sc
        best = min(cands, key=lambda c: abs(c.bus_v - bus_v))
        scale = 1.0 if abs(best.bus_v - bus_v) < 1e-6 else bus_v / best.bus_v
        return best, scale

    def has_assumption(self, field: str) -> bool:
        return any(a.get("field") == field for a in self.assumptions)

    def to_dict(self) -> dict:
        return copy.deepcopy(self.raw)

    def save(self, path: str):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.raw, f, indent=1)
            f.write("\n")

    def __repr__(self):
        return "ActuatorSpec(%r, peak %.4g N m, continuous %.4g N m)" % (self.id, self.peak_nm, self.continuous_nm)


# --------------------------------------------------------------------------------------------- registry

def list_specs() -> Dict[str, str]:
    """{id: path} of the specs shipped with the package (AM units and catalogue examples)."""
    out = {}
    for sub in sorted(os.listdir(SPEC_DIR)):
        p = os.path.join(SPEC_DIR, sub)
        if not os.path.isdir(p):
            continue
        for fn in sorted(os.listdir(p)):
            if fn.endswith(".json"):
                with open(os.path.join(p, fn), "r", encoding="utf-8") as f:
                    out[json.load(f)["id"]] = os.path.join(p, fn)
    return out


def load_spec(id_or_path) -> ActuatorSpec:
    """Load a shipped spec by id ("AM-M", "MyActuator RMD-X8-P20-150-E", ...), a JSON path, or a dict."""
    if isinstance(id_or_path, ActuatorSpec):
        return id_or_path
    if isinstance(id_or_path, dict):
        return ActuatorSpec(id_or_path)
    s = str(id_or_path)
    if os.path.isfile(s):
        with open(s, "r", encoding="utf-8") as f:
            return ActuatorSpec(json.load(f))
    reg = list_specs()
    if s in reg:
        return load_spec(reg[s])
    low = {k.lower(): v for k, v in reg.items()}
    if s.lower() in low:
        return load_spec(low[s.lower()])
    if ("am-" + s).lower() in low:
        return load_spec(low[("am-" + s).lower()])
    raise SpecError("no spec %r; shipped: %s" % (s, ", ".join(reg)))


# --------------------------------------------------------------------------------------------- builder

def spec_from_datasheet(id: str, rated_nm: float, peak_nm: float, mass_kg: float, ratio: Optional[float] = None,
                        rated_speed_rpm: Optional[float] = None, no_load_rpm: Optional[float] = None,
                        bus_v: float = 48.0, reflected_inertia_kgm2: Optional[float] = None,
                        backdrive_nm: Optional[float] = None, gear_efficiency: Optional[float] = None,
                        viscous_nm_s_per_rad: Optional[float] = None, peak_duration_s: Optional[float] = None,
                        ambient_c: Optional[float] = None, cooling: str = "unstated", mount_k_per_w: Optional[float] = None,
                        conditions_text: Optional[str] = None, thermal_tau_s: Optional[float] = None,
                        winding_tau_s: Optional[float] = None, kt_nm_per_a: Optional[float] = None,
                        current_limit_a: Optional[float] = None, phase_resistance_ohm: Optional[float] = None,
                        vendor: str = "", model: str = "", status: str = "", sources: Optional[list] = None,
                        notes: str = "", thermal_model: str = "2node", as_dict: bool = False):
    """Build a spec from minimal datasheet data. Everything not given is filled from DEFAULTS and listed in
    spec['assumptions'] with its basis. Returns an ActuatorSpec (or the dict with as_dict=True).

    Torque-speed envelope when no curves are published: a straight voltage line through the rated point
    (rated_speed, rated_nm) to zero torque at no-load speed; peak curve = min(peak, line), continuous
    curve = min(rated, line). Without any published speed, both curves are flat and no speed limit applies
    (flagged)."""
    d: dict = {"schema": SCHEMA_ID, "id": id, "vendor": vendor, "model": model,
               "status": status or "built from datasheet values by spec_from_datasheet()",
               "sources": sources or [], "notes": notes, "assumptions": []}
    if reflected_inertia_kgm2 is None:
        k, why = DEFAULTS["inertia_per_peak_nm"]
        reflected_inertia_kgm2 = k * peak_nm
        _assume(d, "mechanical.reflected_inertia_kgm2", reflected_inertia_kgm2, why)
    if gear_efficiency is None:
        gear_efficiency, why = DEFAULTS["gear_efficiency"]
        _assume(d, "mechanical.gear_efficiency", gear_efficiency, why)
    coulomb = backdrive_nm
    if coulomb is None:
        k, why = DEFAULTS["coulomb_fraction_of_peak"]
        coulomb = k * peak_nm
        _assume(d, "friction.coulomb_nm", coulomb, why)
    if viscous_nm_s_per_rad is None:
        viscous_nm_s_per_rad, why = DEFAULTS["viscous_nm_s_per_rad"]
        _assume(d, "friction.viscous_nm_s_per_rad", viscous_nm_s_per_rad, why)
    d["mechanical"] = {"ratio": ratio, "mass_kg": mass_kg, "reflected_inertia_kgm2": reflected_inertia_kgm2,
                       "gear_efficiency": gear_efficiency, "backdrive_nm": backdrive_nm}
    if peak_duration_s is None:
        peak_duration_s, why = DEFAULTS["peak_duration_s"]
        _assume(d, "ratings.peak_duration_s", peak_duration_s, why)
    rec, why = DEFAULTS["peak_recovery_s_per_s"]
    _assume(d, "ratings.peak_recovery_s_per_s", rec, why)
    stated = ambient_c is not None or cooling != "unstated"
    if ambient_c is None:
        ambient_c, why = DEFAULTS["ambient_c"]
        _assume(d, "ratings.conditions.ambient_c", ambient_c, why)
    # torque-speed envelope
    w_r = None if rated_speed_rpm is None else rated_speed_rpm * RPM
    w_nl = None if no_load_rpm is None else no_load_rpm * RPM
    curves = []
    if w_r is None and w_nl is None:
        _assume(d, "torque_speed", "flat", "no speed published: peak and continuous curves are flat and no speed "
                                           "limit is modelled")
        w_end = 1.0e3
        curves = [{"bus_v": bus_v, "kind": "peak", "speed_unit": "rad/s", "points": [[0.0, peak_nm], [w_end, peak_nm]]},
                  {"bus_v": bus_v, "kind": "continuous", "speed_unit": "rad/s",
                   "points": [[0.0, rated_nm], [w_end, rated_nm]]}]
    else:
        if w_nl is None:
            k, why = DEFAULTS["no_load_over_rated_speed"]
            w_nl = k * w_r
            _assume(d, "ratings.no_load_speed_rad_s", w_nl, why)
        if w_r is None:
            w_r = 0.0
            _assume(d, "ratings.continuous_speed_rad_s", 0.0, "rated speed not stated; rating taken at stall")
        if w_nl <= w_r:
            raise SpecError("no-load speed must be above rated speed")
        slope = rated_nm / (w_nl - w_r)          # N m per rad/s of the voltage line
        line0 = rated_nm + slope * w_r            # line torque at stall
        if line0 < peak_nm:
            _assume(d, "torque_speed", "line below peak at stall",
                    "the voltage line through the rated point is below the peak torque at stall; peak is held flat "
                    "and then joins the line")

        def env(level):
            corner = max(0.0, (line0 - level) / slope)
            pts = [[0.0, level]]
            if corner > 0.0:
                pts.append([corner, level])
            pts.append([w_nl, 0.0])
            return pts
        curves = [{"bus_v": bus_v, "kind": "peak", "speed_unit": "rad/s", "points": env(peak_nm)},
                  {"bus_v": bus_v, "kind": "continuous", "speed_unit": "rad/s", "points": env(rated_nm)}]
        _assume(d, "torque_speed", "line", "no curve published: straight voltage line through the rated point to "
                                           "zero torque at no-load speed; peak = min(peak, line)")
    d["ratings"] = {
        "peak_nm": peak_nm, "peak_duration_s": peak_duration_s, "peak_recovery_s_per_s": rec,
        "continuous_nm": rated_nm, "continuous_speed_rad_s": w_r, "nominal_bus_v": bus_v,
        "no_load_speed_rad_s": ({str(int(bus_v) if float(bus_v).is_integer() else bus_v): w_nl} if w_nl else {}),
        "conditions": {"stated": bool(stated), "ambient_c": ambient_c, "cooling": cooling,
                       "mount_k_per_w": mount_k_per_w,
                       "text": conditions_text or ("rating conditions not stated" if not stated else "")}}
    d["torque_speed"] = curves
    d["friction"] = {"coulomb_nm": coulomb, "viscous_nm_s_per_rad": viscous_nm_s_per_rad, "stribeck": None}
    d["electrical"] = {"kt_nm_per_a": kt_nm_per_a, "phase_resistance_ohm": phase_resistance_ohm,
                       "current_limit_a": current_limit_a}
    # thermal
    th = {"model": thermal_model, "derate_law": "sqrt"}
    for k in ("rated_winding_c", "derate_c", "trip_c", "trip_hysteresis_k", "copper_tempco_per_k"):
        v, why = DEFAULTS[k]
        th[k] = v
        if k != "copper_tempco_per_k":
            _assume(d, "thermal." + k, v, why)
    th["loss_calibration"] = "electrical" if (kt_nm_per_a and phase_resistance_ohm and ratio) else "rating"
    cs, why = DEFAULTS["c_specific_j_per_kg_k"]
    c_total = cs * mass_kg
    _assume(d, "thermal.nodes.c_total_j_per_k", c_total, why)
    if thermal_tau_s is None:
        thermal_tau_s, why = DEFAULTS["tau_housing_s"]
        _assume(d, "thermal.nodes.tau_housing_s", thermal_tau_s, why)
    if thermal_model == "1node":
        th["nodes"] = {"tau_s": thermal_tau_s, "c_j_per_k": c_total}
    else:
        if winding_tau_s is None:
            winding_tau_s, why = DEFAULTS["tau_winding_s"]
            _assume(d, "thermal.nodes.tau_winding_s", winding_tau_s, why)
        f, why = DEFAULTS["winding_rise_fraction"]
        _assume(d, "thermal.nodes.winding_rise_fraction", f, why)
        th["nodes"] = {"tau_housing_s": thermal_tau_s, "tau_winding_s": winding_tau_s, "winding_rise_fraction": f,
                       "c_total_j_per_k": c_total}
    th["iron_loss"] = {"k1_w_per_rad_s": 0.0, "k2_w_s2_per_rad2": 0.0}
    th["mechanical_losses_heat_actuator"] = True
    d["thermal"] = th
    return d if as_dict else ActuatorSpec(d)


def resolve_thermal_nodes(spec: ActuatorSpec) -> dict:
    """Thermal network in R/C form: {one_node, c_w, r_wh, c_h, r_ha} (J/K, K/W). For 1node, (c_w, r_wh) is the
    single node to ambient and c_h/r_ha are None. Time-constant form uses the approximation that the housing
    mode is tau_h = r_ha * (c_h + c_w) and the winding mode is tau_w = r_wh * c_w."""
    th = spec.thermal
    n = th["nodes"]
    if th["model"] == "1node":
        c = float(n["c_j_per_k"])
        r = float(n["r_k_per_w"]) if "r_k_per_w" in n else float(n["tau_s"]) / c
        return {"one_node": True, "c_w": c, "r_wh": r, "c_h": None, "r_ha": None}
    if "c_winding_j_per_k" in n:
        return {"one_node": False, "c_w": float(n["c_winding_j_per_k"]), "r_wh": float(n["r_winding_housing_k_per_w"]),
                "c_h": float(n["c_housing_j_per_k"]), "r_ha": float(n["r_housing_ambient_k_per_w"])}
    tau_h, tau_w = float(n["tau_housing_s"]), float(n["tau_winding_s"])
    f, c_tot = float(n["winding_rise_fraction"]), float(n["c_total_j_per_k"])
    if not (0.0 < f < 1.0):
        raise SpecError("winding_rise_fraction must be in (0, 1)")
    r_ha = tau_h / c_tot
    r_tot = r_ha / (1.0 - f)
    r_wh = f * r_tot
    c_w = min(tau_w / r_wh, 0.5 * c_tot)
    return {"one_node": False, "c_w": c_w, "r_wh": r_wh, "c_h": c_tot - c_w, "r_ha": r_ha}
