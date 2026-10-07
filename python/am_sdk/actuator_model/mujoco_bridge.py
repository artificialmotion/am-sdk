# SPDX-License-Identifier: Apache-2.0
"""MuJoCo integration: apply ActuatorModel limits and heating per actuator inside an mjData loop.

Two pieces:

* ``mjcf_default_class(spec)`` / ``mjcf_snippet(spec, joint)`` write MJCF for the plant side of the
  actuator: joint armature (reflected inertia), frictionloss (Coulomb), damping (viscous), and a gear=1
  ``<motor>`` whose forcerange is the peak torque. MuJoCo applies these natively and stably.
* ``MujocoActuatorBank(model, data, {joint_name: spec})`` runs one ActuatorModel per joint. Each physics
  step it reads joint velocity, clips your torque command to the speed-, temperature- and budget-dependent
  limit, writes it to the joint's motor actuator (``data.ctrl``) or, if the joint has none, to
  ``data.qfrc_applied``, and advances the thermal state by ``model.opt.timestep``.

    bank = MujocoActuatorBank(model, data, {"hip": "AM-H-T", "knee": "AM-M"})
    while running:
        bank.apply({"hip": tau_hip, "knee": tau_knee})   # or an array in mapping order
        mujoco.mj_step(model, data)

or ``bank.step(cmd)`` for apply + mj_step, or ``bank.install_callback(controller)`` to run inside
mujoco's control callback (the callback is global to the process; one bank at a time).

Works with any MJCF: give the joint -> spec mapping. Joints must be hinge or slide (1 dof).
"""
from __future__ import annotations

from typing import Callable, Dict, Mapping, Optional, Sequence, Union

from .model import ActuatorModel
from .spec import ActuatorSpec, load_spec

try:  # pragma: no cover - import guard
    import mujoco
    HAVE_MUJOCO = True
except Exception:  # noqa: BLE001
    mujoco = None
    HAVE_MUJOCO = False


def _fmt(x: float) -> str:
    return ("%.6g" % x)


def _class_name(spec: ActuatorSpec) -> str:
    return "act_" + "".join(ch if ch.isalnum() else "_" for ch in spec.id)


def mjcf_default_class(spec, class_name: Optional[str] = None, include_friction: bool = True) -> str:
    """A ``<default class=...>`` block for the joint and motor of this actuator (output side, gear=1)."""
    spec = load_spec(spec)
    cn = class_name or _class_name(spec)
    fl = spec.coulomb_nm if include_friction else 0.0
    dmp = spec.viscous if include_friction else 0.0
    pk = spec.peak_nm
    return ('<default class="%s">\n'
            '  <joint armature="%s" frictionloss="%s" damping="%s"/>\n'
            '  <motor gear="1" ctrllimited="true" ctrlrange="-%s %s" forcelimited="true" forcerange="-%s %s"/>\n'
            '</default>' % (cn, _fmt(spec.inertia_kgm2), _fmt(fl), _fmt(dmp), _fmt(pk), _fmt(pk), _fmt(pk), _fmt(pk)))


def mjcf_snippet(spec, joint: str, actuator_name: Optional[str] = None, class_name: Optional[str] = None) -> dict:
    """MJCF pieces for one joint: {"default": <default block>, "joint_attrs": 'class="..."',
    "actuator": '<motor .../>'} plus a header comment with the spec's provenance and assumptions count."""
    spec = load_spec(spec)
    cn = class_name or _class_name(spec)
    an = actuator_name or ("%s_motor" % joint)
    hdr = ("<!-- %s (%s): peak %s N m, continuous %s N m. armature/frictionloss/damping are the plant; "
           "speed, thermal and peak-duration limits come from am_sdk.actuator_model at runtime. "
           "%d assumed values, see the spec. SIMULATED. -->"
           % (spec.id, spec.status or "spec", _fmt(spec.peak_nm), _fmt(spec.continuous_nm), len(spec.assumptions)))
    return {"comment": hdr, "default": mjcf_default_class(spec, cn), "joint_attrs": 'class="%s"' % cn,
            "actuator": '<motor name="%s" class="%s" joint="%s"/>' % (an, cn, joint)}


def mjcf_defaults_block(specs: Sequence) -> str:
    """``<default>...</default>`` with one class per distinct spec, for pasting under <mujoco>."""
    seen, parts = set(), []
    for s in specs:
        s = load_spec(s)
        if s.id in seen:
            continue
        seen.add(s.id)
        parts.append("  " + mjcf_default_class(s).replace("\n", "\n  "))
    return "<default>\n%s\n</default>" % "\n".join(parts)


class MujocoActuatorBank:
    """One ActuatorModel per mapped joint of a MuJoCo model.

    mapping : {joint_name: spec | id | path | ActuatorModel}
    apply_plant_params : copy armature / frictionloss / damping from the spec into the model (default True).
        Set False if your MJCF already has them (e.g. from mjcf_snippet) or you model friction elsewhere.
    explicit_friction : add the spec's friction (incl. Stribeck) as a torque instead of MuJoCo frictionloss.
    model_kwargs : passed to each ActuatorModel (ambient_c, bus_v, peak_policy, cooling_scale, initial_c, ...).
    """

    def __init__(self, model, data, mapping: Mapping[str, Union[str, dict, ActuatorSpec, ActuatorModel]],
                 apply_plant_params: bool = True, explicit_friction: bool = False, **model_kwargs):
        if not HAVE_MUJOCO:
            raise ImportError("mujoco is not importable: pip install mujoco")
        self.model, self.data = model, data
        self.dt = float(model.opt.timestep)
        self.explicit_friction = explicit_friction
        self.names, self.acts, self._dof, self._ctrl, self._gear = [], [], [], [], []
        for jname, s in mapping.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError("no joint %r in the MuJoCo model" % jname)
            if int(model.jnt_type[jid]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise ValueError("joint %r must be a hinge or slide joint" % jname)
            act = s if isinstance(s, ActuatorModel) else ActuatorModel(load_spec(s), dt=self.dt, **model_kwargs)
            dof = int(model.jnt_dofadr[jid])
            # find a motor-style actuator on this joint (transmission = joint, gain fixed, no bias)
            aid, gear = -1, 1.0
            for a in range(model.nu):
                if int(model.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT) and int(model.actuator_trnid[a, 0]) == jid:
                    if int(model.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_NONE):
                        aid, gear = a, float(model.actuator_gear[a, 0]) * float(model.actuator_gainprm[a, 0])
                        break
            if aid >= 0:
                # make sure MuJoCo's own clamps never cut below the model's peak
                pk = act.spec.peak_nm
                g_trn = abs(float(model.actuator_gear[aid, 0]))
                if model.actuator_ctrllimited[aid] and abs(model.actuator_ctrlrange[aid, 1] * gear) < pk:
                    model.actuator_ctrlrange[aid] = [-pk / abs(gear), pk / abs(gear)]
                if model.actuator_forcelimited[aid] and abs(model.actuator_forcerange[aid, 1]) * g_trn < pk:
                    model.actuator_forcerange[aid] = [-pk / g_trn, pk / g_trn]
            if apply_plant_params:
                model.dof_armature[dof] = act.spec.inertia_kgm2
                model.dof_frictionloss[dof] = 0.0 if explicit_friction else act.spec.coulomb_nm
                model.dof_damping[dof] = 0.0 if explicit_friction else act.spec.viscous
            self.names.append(jname)
            self.acts.append(act)
            self._dof.append(dof)
            self._ctrl.append(aid)
            self._gear.append(gear)
        self.index = {n: i for i, n in enumerate(self.names)}
        self.last_tau = [0.0] * len(self.acts)
        self._cb_controller = None

    def __getitem__(self, joint_name: str) -> ActuatorModel:
        return self.acts[self.index[joint_name]]

    def apply(self, cmd) -> list:
        """Clip the commanded joint torques (dict by joint name, or a sequence in mapping order), write them to
        the model and advance each actuator's state by one physics step. Returns the applied torques."""
        d = self.data
        out = []
        for i, act in enumerate(self.acts):
            name = self.names[i]
            if isinstance(cmd, Mapping):
                tc = float(cmd.get(name, 0.0))
            else:
                tc = float(cmd[i])
            dof = self._dof[i]
            w = float(d.qvel[dof])
            tau = act.step(tc, w, self.dt)
            out_tau = tau - (act.friction(w) if self.explicit_friction else 0.0)
            a = self._ctrl[i]
            if a >= 0 and not self.explicit_friction:
                d.ctrl[a] = out_tau / self._gear[i]
            else:
                if a >= 0:
                    d.ctrl[a] = 0.0
                d.qfrc_applied[dof] = out_tau
            out.append(tau)
        self.last_tau = out
        return out

    def step(self, cmd, nstep: int = 1) -> list:
        """apply(cmd) then mj_step, nstep times (the command is held; limits and heating update every step)."""
        out = None
        for _ in range(nstep):
            out = self.apply(cmd)
            mujoco.mj_step(self.model, self.data)
        return out

    def install_callback(self, controller: Callable[[object, object], object]):
        """Run inside MuJoCo's control callback: controller(model, data) -> commands, then apply(). The callback
        is process-global in MuJoCo; call remove_callback() when done."""
        self._cb_controller = controller

        def _cb(m, d):
            self.apply(self._cb_controller(m, d))
        self._cb = _cb
        mujoco.set_mjcb_control(_cb)

    @staticmethod
    def remove_callback():
        mujoco.set_mjcb_control(None)

    def states(self) -> Dict[str, object]:
        return {n: a.state for n, a in zip(self.names, self.acts)}

    def summary(self) -> Dict[str, dict]:
        return {n: a.summary() for n, a in zip(self.names, self.acts)}
