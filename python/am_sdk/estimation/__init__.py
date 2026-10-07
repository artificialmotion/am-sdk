"""Torque estimation and collision detection without an output torque sensor.

SIMULATED, PRE-HARDWARE. Every torque this module returns is an estimate, labelled "estimated", with
an uncertainty bound, a confidence value and a quality flag. No accuracy is claimed before the
evaluation-unit dyno; the figures in docs/ESTIMATION.md are targets, to be confirmed on hardware.

Core (standard library only, 1 kHz-capable):
  EstimatorCalibration   calibration record am_est_cal_v1 (+ JSON schema)
  current_model()        pure function: one sample -> TorqueEstimate
  TorqueEstimator        stateful per-joint estimator (differentiator, quality flags, filter)
  CollisionDetector      momentum-observer residual with threshold and hysteresis (non-safety)
  UkfTorqueEstimator     optional UKF with a learned-residual hook
  LinearFeatureResidual  physics-structured linear residual, adoption_check()
  dither_torque()        optional bounded dither for static estimates (off by default)
Optional: am_sdk.estimation.pinn.PinnResidual (needs numpy, extra [pinn]).
Simulator hook: am_sdk.estimation.simharness.EstimatorHarness.

The C library has the same estimator and detector in am_est.h.
"""
from .calibration import (SCHEMA_ID, SCHEMA_PATH, EfficiencyMap, EstimatorCalibration, load_schema,
                          validate_dict)
from .collision import (CollisionConfig, CollisionDetector, CollisionEvent, CollisionOutput,
                        calibrate_threshold)
from .core import (EST_AMBIGUOUS, EST_DITHER, EST_INVALID, EST_NO_TEMPERATURE, EST_OK, EST_SATURATED, EST_STICK,
                   EstimatorConfig, TorqueEstimate, TorqueEstimator, breakaway_at, current_model,
                   dither_amplitude_max, dither_torque, efficiency_at, friction_at, gear_output, kt_at,
                   quality_str, stick_band_Nm)
from .ukf import (LinearFeatureResidual, UkfTorqueEstimator, adoption_check, baseline_loss, residual_features,
                  residual_targets)

__all__ = [
    "SCHEMA_ID", "SCHEMA_PATH", "EfficiencyMap", "EstimatorCalibration", "load_schema", "validate_dict",
    "CollisionConfig", "CollisionDetector", "CollisionEvent", "CollisionOutput", "calibrate_threshold",
    "EST_AMBIGUOUS", "EST_DITHER", "EST_INVALID", "EST_NO_TEMPERATURE", "EST_OK", "EST_SATURATED", "EST_STICK",
    "EstimatorConfig", "TorqueEstimate", "TorqueEstimator", "breakaway_at", "current_model",
    "dither_amplitude_max", "dither_torque", "efficiency_at", "friction_at", "gear_output", "kt_at",
    "quality_str", "stick_band_Nm",
    "LinearFeatureResidual", "UkfTorqueEstimator", "adoption_check", "baseline_loss", "residual_features",
    "residual_targets",
]
