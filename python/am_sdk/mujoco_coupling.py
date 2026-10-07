# SPDX-License-Identifier: Apache-2.0
"""Optional MuJoCo coupling: drive a MuJoCo model's joints through the same am_* calls.

    import mujoco, am_sdk as am
    from am_sdk.mujoco_coupling import MujocoCoupling

    cfg = am.AmSessionConfig(interface="sim:M,S-D")
    s = am.am_init(cfg); am.am_start(s)
    knee = am.am_open_joint(s, 0)
    cpl = MujocoCoupling(s, model, data, {knee: "knee"})     # joint name in your MJCF
    ...
    while running:
        am.am_set_impedance(knee, ...)      # your controller, unchanged
        cpl.step()                           # one API cycle + physics for the same duration

The coupling marks each mapped sim joint as externally simulated, so MuJoCo is the plant:
joint position and velocity go to the drive model as feedback, and the clamped drive torque
goes into data.qfrc_applied. Torque limits, thermal state, STO and the state machine run
exactly as in the pure sim backend. `apply_plant_params()` copies armature (reflected
inertia) and frictionloss (backdrive) from the unit data into the MuJoCo model, as the
published <UNIT>_mujoco.xml does. Do not also give the joint an MJCF actuator for the same
motion; the API is the actuator.
"""
from __future__ import annotations

from typing import Dict

from .api import AmJoint, AmSession, am_cycle
from .am_types import AmError, AmStatus

try:  # pragma: no cover - import guard
    import mujoco
    HAVE_MUJOCO = True
except Exception:  # noqa: BLE001
    mujoco = None
    HAVE_MUJOCO = False


class MujocoCoupling:
    def __init__(self, session: AmSession, model, data, joint_map: Dict[AmJoint, str], apply_plant_params: bool = True):
        if not HAVE_MUJOCO:
            raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "mujoco is not importable; the sim backend works without it")
        self.session, self.model, self.data = session, model, data
        self.map = []
        for aj, name in joint_map.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise AmError(AmStatus.AM_ERR_PARAM, "no joint %r in the MuJoCo model" % name)
            if int(model.jnt_type[jid]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise AmError(AmStatus.AM_ERR_PARAM, "joint %r must be a hinge or slide joint" % name)
            sj = aj.jb
            if not hasattr(sj, "set_external_state"):
                raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "coupling needs the sim backend")
            sj.opts.external_plant = True
            self.map.append((aj, sj, int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])))
            if apply_plant_params:
                model.dof_armature[model.jnt_dofadr[jid]] = sj.ud.armature_kgm2 + sj.opts.load_inertia_kgm2
                model.dof_frictionloss[model.jnt_dofadr[jid]] = sj.ud.backdrive_nm * sj.opts.friction_scale
        self.substeps = max(1, int(round(session.dt / model.opt.timestep)))
        self._feed()

    def _feed(self):
        for aj, sj, qa, da in self.map:
            sj.set_external_state(float(self.data.qpos[qa]), float(self.data.qvel[da]))

    def step(self) -> int:
        """One API cycle, then physics for one cycle duration. Returns the cycle time in ns."""
        t = am_cycle(self.session)
        for aj, sj, qa, da in self.map:
            self.data.qfrc_applied[da] = sj.torque_applied
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self._feed()
        return t
