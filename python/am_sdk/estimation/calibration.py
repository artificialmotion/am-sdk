"""Estimator calibration record (schema am_est_cal_v1).

One record per joint. It holds the physical model the torque estimator inverts: torque constant
with its temperature coefficient and saturation, friction (Coulomb, Stribeck breakaway, viscous,
output bearing) with temperature terms, gear efficiency forward and reverse (scalars or a
load x temperature grid), the armature inertia and the uncertainty of each term.

Today every record is a design-target or simulated record. End-of-line records measured on a
dyno come with the evaluation units; the fields map onto the calibration objects 0x2200 (Kt),
0x2220 (transmission, friction) and 0x2230 (host estimator model id, version, hash, epoch).

The JSON schema is estimator_calibration.schema.json next to this file.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field, fields
from typing import List, Optional

SCHEMA_ID = "am_est_cal_v1"
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "estimator_calibration.schema.json")
DATA_BASES = ("design_target", "simulated", "measured_type", "measured_unit")
FRICTION_MAP_MAX = 16          # same caps as the C struct (am_est.h)
EFF_TEMPS_MAX = 4
EFF_LOADS_MAX = 4

# Design-target defaults where the unit data has no value (ASSUMED until the EVT dyno).
KT_REF_TEMP_C = 60.0           # unit data Kt is the design value at 60 C magnet temperature
KT_TEMP_COEFF_PER_K = -0.00094 # NdFeB remanence drift (magnet grade data)
COULOMB_OF_BREAKAWAY = 0.7     # Coulomb level as a fraction of the published backdrive torque
STRIBECK_VEL_RAD_S = 0.05
VISCOUS_PER_BREAKAWAY = 0.06   # Nm s/rad per Nm of breakaway (ASSUMED)
REVERSE_LOSS_FACTOR = 2.0      # reverse loss (1 - eta_r) as a multiple of the forward loss (ASSUMED)


@dataclass
class EfficiencyMap:
    """Gear efficiency on a load x temperature grid (bilinear, clamped at the edges)."""
    temps_C: List[float]
    loads_Nm: List[float]
    eta_forward: List[List[float]]    # [temp][load]
    eta_reverse: List[List[float]]


@dataclass
class EstimatorCalibration:
    unit: str
    gear_ratio: float
    kt_Nm_per_A: float                       # motor side, at kt_ref_temp_C
    reflected_inertia_kgm2: float            # armature, output-referred
    rated_torque_Nm: float
    schema: str = SCHEMA_ID
    data_basis: str = "design_target"
    serial: Optional[int] = None
    cal_epoch: int = 0
    date: str = ""
    kt_ref_temp_C: float = KT_REF_TEMP_C
    kt_temp_coeff_per_K: float = KT_TEMP_COEFF_PER_K
    kt_sat_frac: float = 0.0                 # Kt drop at kt_sat_current_A (quadratic in current)
    kt_sat_current_A: float = 0.0            # 0 = no saturation term
    coulomb_Nm: float = 0.0                  # output-referred friction
    breakaway_Nm: float = 0.0                # static (Stribeck) level
    stribeck_vel_rad_s: float = STRIBECK_VEL_RAD_S
    viscous_Nm_s_per_rad: float = 0.0
    friction_ref_temp_C: float = 40.0
    coulomb_temp_coeff_per_K: float = 0.004  # + per K below the reference temperature
    viscous_temp_exp_per_K: float = 0.0225   # exp(c * (Tref - T))
    bearing_Nm: float = 0.0                  # output bearing, downstream of the gear
    friction_map: List[List[float]] = field(default_factory=list)  # [[speed rad/s, friction Nm]], replaces the parametric curve
    eta_forward: float = 1.0
    eta_reverse: float = 1.0
    efficiency_map: Optional[EfficiencyMap] = None
    unc_kt_rel: float = 0.02                 # uncertainty terms used for the bound (about 2 sigma)
    unc_temp_C: float = 6.0
    unc_friction_rel: float = 0.3
    unc_eta_abs: float = 0.02
    unc_inertia_rel: float = 0.1
    note: str = ""

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_unit(cls, unit: str) -> "EstimatorCalibration":
        """Design-target record from the published unit data (am_sdk/data/<UNIT>). Friction split,
        Stribeck velocity, viscous term and reverse efficiency are ASSUMED values."""
        from .. import units as U
        ud = U.load_unit(unit)
        ts = ud.backdrive_nm
        eta_f = ud.gear_eta
        return cls(unit=ud.unit, gear_ratio=ud.ratio, kt_Nm_per_A=ud.kt_motor_nm_per_a,
                   reflected_inertia_kgm2=ud.armature_kgm2, rated_torque_Nm=ud.rated_nm,
                   coulomb_Nm=COULOMB_OF_BREAKAWAY * ts, breakaway_Nm=ts,
                   viscous_Nm_s_per_rad=VISCOUS_PER_BREAKAWAY * ts,
                   eta_forward=eta_f, eta_reverse=max(0.5, 1.0 - REVERSE_LOSS_FACTOR * (1.0 - eta_f)),
                   note="design target from published unit data; friction shape and reverse efficiency assumed")

    @classmethod
    def from_joint(cls, joint) -> "EstimatorCalibration":
        """Record from the joint's calibration read-out (am_read_calibration). On the simulator this
        gives the design values; measured records come with the evaluation units."""
        from ..api import am_read_calibration
        raw = json.loads(am_read_calibration(joint))
        cal = cls.from_unit(raw["unit"])
        cal.gear_ratio = float(raw.get("gear_ratio", cal.gear_ratio))
        cal.kt_Nm_per_A = float(raw.get("kt_motor_Nm_per_A", cal.kt_Nm_per_A))
        cal.reflected_inertia_kgm2 = float(raw.get("reflected_inertia_kgm2", cal.reflected_inertia_kgm2))
        if raw.get("data_basis") == "design_target":
            cal.data_basis = "design_target"
        return cal

    # ------------------------------------------------------------------ JSON
    def to_dict(self) -> dict:
        d = asdict(self)
        if self.efficiency_map is None:
            d["efficiency_map"] = None
        return d

    def to_json(self, indent: int = 1) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    @classmethod
    def from_dict(cls, d: dict) -> "EstimatorCalibration":
        errs = validate_dict(d)
        if errs:
            raise ValueError("invalid calibration record: " + "; ".join(errs))
        names = {f.name for f in fields(cls)}
        kw = {k: v for k, v in d.items() if k in names}
        em = kw.get("efficiency_map")
        if em is not None:
            kw["efficiency_map"] = EfficiencyMap(**em)
        return cls(**kw)

    @classmethod
    def from_json(cls, text: str) -> "EstimatorCalibration":
        return cls.from_dict(json.loads(text))

    def validate(self) -> List[str]:
        return validate_dict(self.to_dict())


_NUM_POS = ("gear_ratio", "kt_Nm_per_A", "reflected_inertia_kgm2", "rated_torque_Nm", "stribeck_vel_rad_s")
_NUM_NONNEG = ("kt_sat_frac", "kt_sat_current_A", "coulomb_Nm", "breakaway_Nm", "viscous_Nm_s_per_rad", "bearing_Nm",
               "unc_kt_rel", "unc_temp_C", "unc_friction_rel", "unc_eta_abs", "unc_inertia_rel")
_NUM_ANY = ("kt_ref_temp_C", "kt_temp_coeff_per_K", "friction_ref_temp_C", "coulomb_temp_coeff_per_K",
            "viscous_temp_exp_per_K")


def _isnum(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def validate_dict(d: dict) -> List[str]:
    """Checks a record against the am_est_cal_v1 rules (the same rules as the JSON schema, plus the
    cross-field ones a schema cannot express). Returns a list of problems; empty means valid."""
    e = []
    if not isinstance(d, dict):
        return ["record is not an object"]
    if d.get("schema") != SCHEMA_ID:
        e.append("schema must be %r" % SCHEMA_ID)
    if not isinstance(d.get("unit"), str) or not d.get("unit"):
        e.append("unit must be a non-empty string")
    if d.get("data_basis", "design_target") not in DATA_BASES:
        e.append("data_basis must be one of %s" % ", ".join(DATA_BASES))
    for k in _NUM_POS:
        if not _isnum(d.get(k)) or d[k] <= 0:
            e.append("%s must be a number > 0" % k)
    for k in _NUM_NONNEG:
        if k in d and (not _isnum(d[k]) or d[k] < 0):
            e.append("%s must be a number >= 0" % k)
    for k in _NUM_ANY:
        if k in d and not _isnum(d[k]):
            e.append("%s must be a number" % k)
    for k in ("eta_forward", "eta_reverse"):
        if k in d and (not _isnum(d[k]) or not 0.3 <= d[k] <= 1.0):
            e.append("%s must be within 0.3..1" % k)
    if _isnum(d.get("kt_sat_frac")) and d["kt_sat_frac"] >= 0.5:
        e.append("kt_sat_frac must be below 0.5")
    if _isnum(d.get("coulomb_Nm")) and _isnum(d.get("breakaway_Nm")) and d["coulomb_Nm"] > d["breakaway_Nm"] + 1e-12:
        e.append("coulomb_Nm must not exceed breakaway_Nm")
    fm = d.get("friction_map") or []
    if not isinstance(fm, list) or len(fm) > FRICTION_MAP_MAX:
        e.append("friction_map must be a list of at most %d points" % FRICTION_MAP_MAX)
    else:
        prev = -1.0
        for p in fm:
            if not (isinstance(p, (list, tuple)) and len(p) == 2 and _isnum(p[0]) and _isnum(p[1])
                    and p[0] > prev and p[0] >= 0 and p[1] >= 0):
                e.append("friction_map points must be [speed >= 0 increasing, friction >= 0]")
                break
            prev = p[0]
    em = d.get("efficiency_map")
    if em is not None:
        try:
            nt, nl = len(em["temps_C"]), len(em["loads_Nm"])
            ok = 1 <= nt <= EFF_TEMPS_MAX and 1 <= nl <= EFF_LOADS_MAX
            ok = ok and all(_isnum(x) for x in em["temps_C"] + em["loads_Nm"])
            ok = ok and all(b > a for a, b in zip(em["temps_C"], em["temps_C"][1:]))
            ok = ok and all(b > a for a, b in zip(em["loads_Nm"], em["loads_Nm"][1:]))
            for key in ("eta_forward", "eta_reverse"):
                g = em[key]
                ok = ok and len(g) == nt and all(len(r) == nl and all(_isnum(v) and 0.3 <= v <= 1.0 for v in r) for r in g)
            if not ok:
                e.append("efficiency_map must be a %dx%d grid at most, axes increasing, eta within 0.3..1"
                         % (EFF_TEMPS_MAX, EFF_LOADS_MAX))
        except (KeyError, TypeError):
            e.append("efficiency_map needs temps_C, loads_Nm, eta_forward, eta_reverse")
    return e


def load_schema() -> dict:
    with open(SCHEMA_PATH, "r", encoding="utf-8") as f:
        return json.load(f)
