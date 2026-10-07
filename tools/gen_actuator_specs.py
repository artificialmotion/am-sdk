"""Generate the actuator-model spec files shipped in python/am_sdk/actuator_model/specs/.

    python tools/gen_actuator_specs.py

AM units: from the per-unit data in python/am_sdk/data/<UNIT>/<UNIT>_params.json (the site downloads),
the motor design constants in am_sdk.units, and the declared rating mount from the site's published
fit data. Thermal values the data does not give are the SDK simulator's model values and are listed
as assumptions in each spec.

Catalogue units: face values exactly as published by the seller or reseller (source URL and access
date in each file). Rating conditions are not stated by those sources; every value the model needs
beyond them comes from spec_from_datasheet() defaults and is listed in "assumptions".
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "python"))

from am_sdk import units as U  # noqa: E402
from am_sdk.actuator_model.spec import (DEFAULTS, RPM, SCHEMA_ID, SPEC_DIR, ActuatorSpec,  # noqa: E402
                                        spec_from_datasheet)

# Declared rating mount (K/W), published with the rated point on artificialmotion.com (fit data, "rated.mount").
AM_MOUNT_K_PER_W = {"S-S": 0.732422, "S-D": 0.732422, "M": 0.3, "H-F": 0.2083, "H-T": 0.2083, "H-X": 0.1722}

SIM_NOTE = "AM SDK simulator model value (am_sdk/backends/sim.py), not a published rating"


def am_spec(unit: str) -> dict:
    with open(os.path.join(ROOT, "python", "am_sdk", "data", unit, "%s_params.json" % unit), "r", encoding="utf-8") as f:
        pj = json.load(f)
    p = {k: v["value"] for k, v in pj["parameters"].items()}
    kt, eta, kt_src = U.MOTOR_DESIGN[unit]
    d = {"schema": SCHEMA_ID, "id": "AM-%s" % unit, "vendor": "Artificial Motion", "model": unit,
         "status": "design target; SIMULATED, not built. " + pj.get("status", ""),
         "data_revision": pj.get("revision", ""),
         "sources": [{"url": "https://artificialmotion.com", "accessed": "2026-10-07",
                      "note": "per-unit download %s_params.json, copied in am_sdk/data/%s/" % (unit, unit)}],
         "notes": "", "assumptions": []}
    notes = ["Continuous rating is the typical unit; end-of-life continuous %.4g N m." %
             p["rated_torque_continuous_end_of_life"]]
    if p.get("continuous_clamp"):
        notes.append("Published continuous clamp %.4g N m." % p["continuous_clamp"])
    notes.append("Peak: 2 s event from a warm start.")
    d["notes"] = " ".join(notes)
    d["mechanical"] = {"ratio": p["gear_ratio"], "mass_kg": p["mass"],
                       "reflected_inertia_kgm2": p["output_inertia_armature"], "gear_efficiency": eta,
                       "backdrive_nm": p["friction_backdrive"]}
    d["ratings"] = {"peak_nm": p["peak_torque_2s"], "peak_duration_s": 2.0,
                    "peak_recovery_s_per_s": U.PEAK_RECHARGE_S_PER_S,
                    "continuous_nm": p["rated_torque_continuous_typical"],
                    "continuous_speed_rad_s": p["rated_speed"] * RPM, "nominal_bus_v": 48,
                    "no_load_speed_rad_s": {"48": p["no_load_speed_48V"], "36": p["no_load_speed_36V"]},
                    "conditions": {"stated": True, "ambient_c": 25.0, "cooling": "bolted_structure",
                                   "mount_k_per_w": AM_MOUNT_K_PER_W[unit],
                                   "text": "25 C ambient, still air, bolted to the robot structure (declared mount "
                                           "%g K/W), 48 V" % AM_MOUNT_K_PER_W[unit]}}
    d["assumptions"].append({"field": "ratings.peak_recovery_s_per_s", "value": U.PEAK_RECHARGE_S_PER_S,
                             "basis": SIM_NOTE})
    curves = []
    for key, c in pj["curves"].items():
        kind, bus = key.split("_")
        curves.append({"bus_v": float(bus.rstrip("V")), "kind": kind, "speed_unit": "rpm",
                       "points": c["points"]})
    d["torque_speed"] = curves
    d["friction"] = {"coulomb_nm": p["friction_backdrive"], "viscous_nm_s_per_rad": 0.0, "stribeck": None}
    d["assumptions"].append({"field": "friction.coulomb_nm", "value": p["friction_backdrive"],
                             "basis": "published unpowered backdrive torque used as Coulomb friction"})
    d["assumptions"].append({"field": "friction.viscous_nm_s_per_rad", "value": 0.0,
                             "basis": "viscous damping not specified in the published data; 0"})
    d["electrical"] = {"kt_nm_per_a": kt, "kt_basis": kt_src, "phase_resistance_ohm": None, "current_limit_a": None}
    tau = p.get("thermal_time_constant")
    c_tot = DEFAULTS["c_specific_j_per_kg_k"][0] * p["mass"]
    th = {"model": "1node", "loss_calibration": "rating", "rated_winding_c": 105.0, "derate_c": 105.0,
          "trip_c": 130.0, "trip_hysteresis_k": 5.0, "derate_law": "sqrt",
          "copper_tempco_per_k": DEFAULTS["copper_tempco_per_k"][0],
          "nodes": {"tau_s": tau if tau is not None else U.FAMILY_THERMAL_TAU_S, "c_j_per_k": c_tot},
          "iron_loss": {"k1_w_per_rad_s": 0.0, "k2_w_s2_per_rad2": 0.0},
          "mechanical_losses_heat_actuator": True}
    d["thermal"] = th
    if tau is None:
        d["assumptions"].append({"field": "thermal.nodes.tau_s", "value": U.FAMILY_THERMAL_TAU_S,
                                 "basis": "no published time constant for this unit; conservative family value "
                                          "(shortest published family value), as in the SDK simulator"})
    else:
        d["assumptions"].append({"field": "thermal.nodes.tau_s", "value": tau,
                                 "basis": "published thermal time constant (dominant passive mode, rated mount)"})
    d["assumptions"].append({"field": "thermal.nodes.c_j_per_k", "value": c_tot,
                             "basis": DEFAULTS["c_specific_j_per_kg_k"][1] + "; scales watts only"})
    for k, v in (("rated_winding_c", 105.0), ("derate_c", 105.0), ("trip_c", 130.0), ("trip_hysteresis_k", 5.0)):
        d["assumptions"].append({"field": "thermal." + k, "value": v, "basis": SIM_NOTE})
    ActuatorSpec(d)  # validate
    return d


# Face values exactly as published (seller or reseller tables), accessed 2026-10-04 / 2026-10-07.
CATALOGUE = [
    dict(id="MyActuator-RMD-X8-P20-150-E", vendor="MyActuator", model="RMD-X8-P20-150-E", rated_nm=43, peak_nm=120,
         rated_speed_rpm=127, mass_kg=1.33, bus_v=48,
         url="https://myactuator.us/products/rmd-x-series/rmd-x8-p20-150-e-03", accessed="2026-10-04",
         note="Seller table: rated at 127 rpm, 48 V. Seller rating conditions (mount, ambient, duration) not stated."),
    dict(id="MyActuator-RMD-X12-320", vendor="MyActuator", model="RMD-X12-320", rated_nm=85, peak_nm=320,
         rated_speed_rpm=100, no_load_rpm=125, mass_kg=2.37, bus_v=48, ratio=20,
         url="https://aifitlab.com/products/myactuator-rmd-x12-320-motor", accessed="2026-10-04",
         note="Reseller technical table: 85 Nm rated at 100 rpm, 20:1, 30 A rms rated phase current. "
              "Rating conditions not stated."),
    dict(id="CubeMars-AK60-39-V3", vendor="CubeMars", model="AK60-39 V3", rated_nm=24, peak_nm=72, rated_speed_rpm=70,
         mass_kg=0.75, bus_v=48, url="https://www.cubemars.com/product/ak60-39-v3-0-kv80-robotic-actuator.html",
         accessed="2026-10-04",
         note="Seller page: rated at 70 rpm, 48 V. Seller rating conditions (mount, ambient, duration) not stated."),
    dict(id="RobStride-04", vendor="RobStride", model="RobStride 04", rated_nm=40, peak_nm=120, rated_speed_rpm=167,
         no_load_rpm=200, mass_kg=1.42, bus_v=48, ratio=9,
         url="https://www.seeedstudio.com/Robostride-04-Actuator-p-6775.html", accessed="2026-10-04",
         note="Reseller listing: 40 Nm rated, 120 Nm peak, 9:1, 1.42 kg. Rating conditions not stated."),
    dict(id="Silica-Q-13", vendor="Silica", model="Q-13", rated_nm=19, peak_nm=40, mass_kg=1.236, bus_v=48,
         url="https://www.silicamachines.com/buy", accessed="2026-10-07",
         note="Seller specification dialog, \"design targets\"; continuous torque 19 N m \"mounted\". Rating "
              "conditions (mount, ambient, speed) not stated; no speed published."),
]


def catalogue_spec(row: dict) -> dict:
    r = dict(row)
    src = [{"url": r.pop("url"), "accessed": r.pop("accessed"), "note": r["note"]}]
    note = r.pop("note")
    d = spec_from_datasheet(status="catalogue face value as published; rating conditions unstated; not verified by "
                                   "Artificial Motion", sources=src,
                            notes=note + " Model values beyond the published face values are assumptions (see "
                                         "'assumptions').",
                            conditions_text="rating conditions not stated by the source", as_dict=True, **r)
    ActuatorSpec(d)
    return d


def main():
    out = []
    os.makedirs(os.path.join(SPEC_DIR, "am"), exist_ok=True)
    os.makedirs(os.path.join(SPEC_DIR, "catalogue"), exist_ok=True)
    for u in U.UNIT_IDS:
        d = am_spec(u)
        p = os.path.join(SPEC_DIR, "am", "%s.json" % d["id"])
        out.append((d, p))
    for row in CATALOGUE:
        d = catalogue_spec(row)
        p = os.path.join(SPEC_DIR, "catalogue", "%s.json" % d["id"])
        out.append((d, p))
    for d, p in out:
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            json.dump(d, f, indent=1)
            f.write("\n")
        print("wrote", os.path.relpath(p, ROOT))


if __name__ == "__main__":
    main()
