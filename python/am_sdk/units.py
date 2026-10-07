"""Per-unit plant data: published parameters and torque-speed limits.

Data comes from the per-unit sim packages on the site downloads (copied into
am_sdk/data/<UNIT>/). The published am_<unit>_model.py supplies torque_limit()
and ThermalState; <UNIT>_params.json supplies inertia, ratio and friction.
All values are design targets.
Kt60 values are the design torque constants at 60 C magnet temperature.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import warnings
from dataclasses import dataclass
from fractions import Fraction

from .am_types import AmSku

UNIT_IDS = ("S-S", "S-D", "M", "H-F", "H-T", "H-X")
_SKU = {"S-D": AmSku.S_D, "S-S": AmSku.S_S, "M": AmSku.M, "H-F": AmSku.H_F, "H-T": AmSku.H_T, "H-X": AmSku.H_X}
_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

# Simulator model values (not published ratings):
FAMILY_THERMAL_TAU_S = 210.0       # model files use this where the unit has no published tau

# Per-unit motor torque constant Kt (N*m per A, peak-referred) and nominal gear efficiency, from the
# motor design data of each unit. Torque estimate: kt * iq * ratio * gear_eta.
# unit: (kt, gear_eta, source)
MOTOR_DESIGN = {
    "S-D": (0.0924, 0.965, "AM motor design data, S-D (peak-referred Kt; nominal gear efficiency)"),
    "S-S": (0.0862, 0.935, "AM motor design data, S-S (peak-referred Kt; nominal gear efficiency)"),
    "M": (0.136916, 0.926, "AM motor design data, M (Kt at 60 C; nominal gear efficiency)"),
    "H-F": (0.123224, 0.9194, "AM motor design data, H-F (Kt at 60 C; nominal gear efficiency)"),
    "H-T": (0.146695, 0.9265, "AM motor design data, H-T (Kt at 60 C; nominal gear efficiency)"),
    "H-X": (0.13853, 0.926, "AM motor design data, H-X (Kt at 60 C; nominal gear efficiency)"),
}
WINDING_RISE_AT_RATED_K = 80.0     # K above ambient at x = 1 (continuous thermal limit)
PEAK_BUDGET_S = 2.0                # published: 2 s peak event from a warm start
PEAK_RECHARGE_S_PER_S = 0.1        # budget recovery (sim model value: 2 s of peak per 20 s)
PEAK_REARM_S = 0.5                 # peak is released again once the budget is back to this
BUS_SOURCE_R_OHM = 0.05            # supply sag model


@dataclass
class UnitData:
    unit: str
    sku: AmSku
    ratio: float
    armature_kgm2: float
    mass_kg: float
    peak_nm: float
    rated_nm: float
    rated_speed_rad_s: float
    no_load_rad_s: dict
    backdrive_nm: float
    thermal_tau_s: float
    thermal_tau_published: bool
    kt_motor_nm_per_a: float
    gear_eta: float
    kt_source: str
    params: dict
    model: object  # the published am_<unit>_model module

    @property
    def mujoco_xml(self) -> str:
        return os.path.join(_DATA_DIR, self.unit, "%s_mujoco.xml" % self.unit)


_CACHE = {}


def _load_model(unit: str):
    fn = "am_%s_model.py" % unit.lower().replace("-", "_")
    path = os.path.join(_DATA_DIR, unit, fn)
    name = "am_sdk_data_" + unit.lower().replace("-", "_")
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def normalize_unit(unit: str) -> str:
    u = unit.strip().upper().replace("_", "-")
    if u in UNIT_IDS:
        return u
    raise ValueError("unknown unit %r; choose one of %s" % (unit, ", ".join(UNIT_IDS)))


def load_unit(unit: str) -> UnitData:
    unit = normalize_unit(unit)
    if unit in _CACHE:
        return _CACHE[unit]
    with open(os.path.join(_DATA_DIR, unit, "%s_params.json" % unit), "r", encoding="utf-8") as f:
        params = json.load(f)
    p = {k: v["value"] for k, v in params["parameters"].items()}
    model = _load_model(unit)
    rpm = 2.0 * math.pi / 60.0
    tau = p.get("thermal_time_constant")
    published = tau is not None
    if tau is None:
        tau = FAMILY_THERMAL_TAU_S
    kt, eta, kt_src = MOTOR_DESIGN[unit]
    ud = UnitData(
        unit=unit, sku=_SKU[unit], ratio=float(p["gear_ratio"]),
        armature_kgm2=float(p["output_inertia_armature"]), mass_kg=float(p["mass"]),
        peak_nm=float(p["peak_torque_2s"]), rated_nm=float(p["rated_torque_continuous_typical"]),
        rated_speed_rad_s=float(p["rated_speed"]) * rpm,
        no_load_rad_s={"48": float(p["no_load_speed_48V"]), "36": float(p["no_load_speed_36V"])},
        backdrive_nm=float(p["friction_backdrive"] or 0.0),
        thermal_tau_s=float(tau), thermal_tau_published=published,
        kt_motor_nm_per_a=kt, gear_eta=eta, kt_source=kt_src,
        params=params, model=model)
    _CACHE[unit] = ud
    return ud


def ratio_fraction(ratio: float):
    f = Fraction(ratio).limit_denominator(1000)
    return f.numerator, f.denominator


def new_thermal(ud: UnitData, x0: float = 0.0):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return ud.model.ThermalState(tau_s=ud.thermal_tau_s, cont_torque=ud.rated_nm, x0=x0)
