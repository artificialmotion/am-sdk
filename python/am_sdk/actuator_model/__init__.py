"""Open, thermal-aware actuator model for any actuator (AM units or catalogue units).

Spec (JSON) -> runtime model (torque limit vs speed and temperature, losses, 1- or 2-node thermal
network, peak budget, derate, trip, duty cycle, events) -> MuJoCo bank or Isaac Lab actuator.
Stdlib core; numpy or torch only for the batched model. Everything is a simulation model.

    from am_sdk.actuator_model import load_spec, ActuatorModel, spec_from_datasheet, list_specs
"""
from .spec import (SCHEMA_ID, DEFAULTS, ActuatorSpec, Curve, SpecError, list_specs, load_spec, resolve_thermal_nodes,
                   spec_from_datasheet, validate_spec)
from .model import (ActuatorModel, ActuatorState, DutyTracker, Event, friction_torque, SATURATION_START,
                    SATURATION_END, PEAK_START, PEAK_END, PEAK_BUDGET_EMPTY, PEAK_REARMED, DERATE_START, DERATE_END,
                    TRIP, TRIP_CLEAR)
from .batch import BatchActuatorModel

__all__ = ["SCHEMA_ID", "DEFAULTS", "ActuatorSpec", "Curve", "SpecError", "list_specs", "load_spec",
           "resolve_thermal_nodes", "spec_from_datasheet", "validate_spec", "ActuatorModel", "ActuatorState",
           "DutyTracker", "Event", "friction_torque", "BatchActuatorModel", "SATURATION_START", "SATURATION_END",
           "PEAK_START", "PEAK_END", "PEAK_BUDGET_EMPTY", "PEAK_REARMED", "DERATE_START", "DERATE_END", "TRIP",
           "TRIP_CLEAR"]
