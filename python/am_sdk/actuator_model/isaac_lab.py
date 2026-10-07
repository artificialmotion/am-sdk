# SPDX-License-Identifier: Apache-2.0
"""Isaac Lab actuator: thermal-aware torque limits for any spec, as an ActuatorBase subclass.

UNTESTED ON ISAAC LAB. Written against the documented Isaac Lab 2.x API
(``isaaclab.actuators.IdealPDActuator`` / ``IdealPDActuatorCfg``, ``isaaclab.utils.configclass``,
``ArticulationActions`` with joint_positions / joint_velocities / joint_efforts). The numeric core is
``BatchActuatorModel`` (torch backend), which is unit-tested; the glue is tested against a stub of that
API only. Check field names against your Isaac Lab version (older releases: ``omni.isaac.lab``).

    from am_sdk.actuator_model.isaac_lab import make_actuator_cfg
    ROBOT_CFG.actuators = {
        "knees": make_actuator_cfg("AM-M", joint_names_expr=[".*_knee"], stiffness=150.0, damping=6.0,
                                   physics_dt=sim_cfg.dt),
        "hips": make_actuator_cfg({"hip_pitch": "AM-H-T", "hip_roll": "AM-H-F"},
                                  joint_names_expr=[".*_hip_.*"], stiffness=200.0, damping=8.0,
                                  physics_dt=sim_cfg.dt),
    }

Isaac Lab calls ``compute()`` every physics step, so ``physics_dt`` is the physics step (sim.dt), not the
policy step. The PD law matches IdealPDActuator; the effort is then clipped by the speed-, temperature-
and peak-budget-dependent limit, and the thermal state advances. ``effort_limit_sim`` is set to the peak
torque so PhysX never clips below the model. Per-env state: ``actuator.model.tw`` (winding C),
``.derate``, ``.budget``, ``.tripped``, ``.counts``.
"""
from __future__ import annotations

import re
from typing import Dict, Optional, Sequence, Union

from .batch import BatchActuatorModel
from .spec import load_spec

_CLASSES = None


def _import_isaaclab():
    try:
        from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
        from isaaclab.utils import configclass
        return IdealPDActuator, IdealPDActuatorCfg, configclass
    except ImportError:
        pass
    try:  # older releases
        from omni.isaac.lab.actuators import IdealPDActuator, IdealPDActuatorCfg
        from omni.isaac.lab.utils import configclass
        return IdealPDActuator, IdealPDActuatorCfg, configclass
    except ImportError as e:
        raise ImportError("Isaac Lab is not importable (isaaclab / omni.isaac.lab). The pure-Python core "
                          "am_sdk.actuator_model.BatchActuatorModel works without it.") from e


def resolve_joint_specs(spec_or_map, joint_names: Sequence[str]):
    """Per-joint specs for the joints this actuator group drives. spec_or_map is one spec (all joints) or
    {regex: spec}; the first regex that fully matches a joint name wins."""
    if isinstance(spec_or_map, dict) and "schema" not in spec_or_map:
        out = []
        for jn in joint_names:
            for pat, s in spec_or_map.items():
                if re.fullmatch(pat, jn):
                    out.append(load_spec(s))
                    break
            else:
                raise ValueError("no spec pattern matches joint %r" % jn)
        return out
    s = load_spec(spec_or_map)
    return [s] * len(joint_names)


def _env_ids(env_ids, num_envs):
    if env_ids is None:
        return None
    if isinstance(env_ids, slice) and env_ids == slice(None):
        return None
    return env_ids


def _classes():
    global _CLASSES
    if _CLASSES is not None:
        return _CLASSES
    IdealPDActuator, IdealPDActuatorCfg, configclass = _import_isaaclab()

    class AmThermalActuator(IdealPDActuator):
        """IdealPD + am_sdk.actuator_model limits (speed curve, thermal derate/trip, peak budget)."""

        def __init__(self, cfg, *args, **kwargs):
            super().__init__(cfg, *args, **kwargs)
            specs = resolve_joint_specs(cfg.spec, list(self.joint_names))
            self.model = BatchActuatorModel(specs, num_envs=self._num_envs, dt=cfg.physics_dt, backend="torch",
                                            device=self._device, ambient_c=cfg.ambient_c, bus_v=cfg.bus_v,
                                            peak_policy=cfg.peak_policy, cooling_scale=cfg.cooling_scale,
                                            initial_c=cfg.initial_c)

        def reset(self, env_ids):
            super().reset(env_ids)
            self.model.reset(_env_ids(env_ids, self._num_envs), initial_c=self.cfg.initial_c)

        def compute(self, control_action, joint_pos, joint_vel):
            error_pos = control_action.joint_positions - joint_pos
            error_vel = control_action.joint_velocities - joint_vel
            self.computed_effort = self.stiffness * error_pos + self.damping * error_vel + control_action.joint_efforts
            self.applied_effort = self.model.step(self.computed_effort, joint_vel)
            control_action.joint_efforts = self.applied_effort
            control_action.joint_positions = None
            control_action.joint_velocities = None
            return control_action

    @configclass
    class AmThermalActuatorCfg(IdealPDActuatorCfg):
        class_type: type = AmThermalActuator
        spec: object = None              # one spec / id / path, or {joint regex: spec}
        physics_dt: float = 0.005        # set to sim.dt
        ambient_c: Optional[float] = None
        bus_v: Optional[float] = None
        peak_policy: str = "budget"
        cooling_scale: float = 1.0
        initial_c: Optional[float] = None

    _CLASSES = (AmThermalActuator, AmThermalActuatorCfg)
    return _CLASSES


def make_actuator_cfg(spec_or_map: Union[str, dict, object], joint_names_expr: Sequence[str], stiffness=0.0,
                      damping=0.0, physics_dt: float = 0.005, **kwargs):
    """AmThermalActuatorCfg for an actuator group. armature, friction, effort and velocity limits are filled
    from the spec (for a {regex: spec} map, as {regex: value} dicts). Extra kwargs go to the cfg."""
    _, Cfg = _classes()
    if isinstance(spec_or_map, dict) and "schema" not in spec_or_map:
        specs: Dict[str, object] = {k: load_spec(v) for k, v in spec_or_map.items()}
        per = lambda f: {k: f(s) for k, s in specs.items()}  # noqa: E731
        spec_arg = specs
    else:
        s = load_spec(spec_or_map)
        per = lambda f: f(s)  # noqa: E731
        spec_arg = s

    def vlim(sp):
        nl = sp.no_load.get(sp.nominal_bus_v)
        return float(nl) if nl else sp.curve("peak")[0].w_max

    fields = dict(joint_names_expr=list(joint_names_expr), stiffness=stiffness, damping=damping,
                  armature=per(lambda sp: sp.inertia_kgm2), friction=per(lambda sp: sp.coulomb_nm),
                  effort_limit=per(lambda sp: sp.peak_nm), effort_limit_sim=per(lambda sp: sp.peak_nm),
                  velocity_limit=per(vlim), spec=spec_arg, physics_dt=physics_dt)
    fields.update(kwargs)
    return Cfg(**fields)
