# SPDX-License-Identifier: Apache-2.0
"""Session and handle API. Function names mirror c/include/am_api.h.

Python conventions: functions return their output value directly and raise AmError
(with the C status in .status) where the C function returns a negative am_status.
"""
from __future__ import annotations

import copy
import math
import time
from typing import List, Optional, Tuple

from . import codec
from .backends.base import Backend, JointBackend
from .backends.sim import SimBackend
from .backends.transport import TransportBackend
from .am_types import (API_VERSION, AM_CAP_MODE_CURRENT, FAULT_NAMES, OD_MAJOR_SUPPORTED, AmBus,
                    AmCommand, AmDeviceInfo, AmDriveState, AmError, AmEvent, AmEventKind, AmFaultInfo,
                    AmFaultRecord, AmIdentity, AmImpedance, AmLimits, AmLoopGains, AmMode, AmRefState,
                    AmSessionConfig, AmStatus, AmTelemetry, AmTransparency, am_fault_name, am_status_str,
                    am_version_pack)

SANITY_TORQUE_NM = 1.0e4
SANITY_RATE = 1.0e4
SANITY_Q = 1.0e6
SANITY_GAIN = 1.0e7



class AmSession:
    """One bus connection (am_session)."""

    def __init__(self, cfg: AmSessionConfig, backend: Backend):
        self.cfg = cfg
        self.backend = backend
        self.joints: List["AmJoint"] = []
        self.started = False
        self.cycle_idx = 0
        self.t_ns = 0
        self.closed = False
        self.log: List[str] = []

    @property
    def dt(self) -> float:
        return self.cfg.cycle_us * 1e-6


class AmJoint:
    """One actuator on a session (am_joint). Owned by the session."""

    def __init__(self, session: AmSession, address: int, jb: JointBackend):
        self.session = session
        self.address = address
        self.jb = jb
        self.mode = AmMode.NONE
        self.cmd = AmCommand()
        self.cmd_fresh = False
        self.enable_req = False
        self.quick_stop_req = False
        self.reset_req = False
        self.tlm: Optional[AmTelemetry] = None
        self.tlm_cycle = -10**9
        self.identity = jb.identity()
        self.transparency = jb.transparency()
        self.limits_cache = jb.get_limits()[0]
        self.od_checked = False

    # convenience for tests and tools; not part of the C API
    @property
    def state(self) -> AmDriveState:
        return self.tlm.state if self.tlm is not None else AmDriveState.NOT_READY_TO_SWITCH_ON


def _chk(cond: bool, status: AmStatus, detail: str = ""):
    if not cond:
        raise AmError(status, detail)


def _finite(*vals):
    for v in vals:
        if v is None or isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise AmError(AmStatus.AM_ERR_PARAM, "non-finite or non-numeric value")


# ---------------------------------------------------------------- lifecycle
def am_api_version() -> int:
    """SDK API version packed as AM_VERSION_PACK (8.8.16 bits)."""
    return am_version_pack(*API_VERSION)


def _backend_for(cfg: AmSessionConfig) -> Backend:
    if cfg.backend is not None:
        return cfg.backend
    iface = cfg.interface or "sim"
    if iface == "sim" or iface.startswith("sim:"):
        return SimBackend()
    if iface.startswith("ecat:"):
        from .backends.ethercat import EtherCatTransport
        _chk(cfg.bus == AmBus.ETHERCAT, AmStatus.AM_ERR_PARAM, "ecat: interface needs bus=AmBus.ETHERCAT")
        return TransportBackend(EtherCatTransport(iface[5:]))
    if iface.startswith("can:"):
        from .backends.canfd import CanFdTransport, parse_can_interface
        _chk(cfg.bus == AmBus.CANFD, AmStatus.AM_ERR_PARAM, "can: interface needs bus=AmBus.CANFD")
        i, ch, nom, dat = parse_can_interface(iface)
        return TransportBackend(CanFdTransport(i, ch, nom, dat))
    raise AmError(AmStatus.AM_ERR_PARAM, "unknown interface %r: use sim[:units], ecat:<nic> or "
                                         "can:<interface>:<channel>[@nominal/data]" % iface)


def am_init(cfg: Optional[AmSessionConfig] = None) -> AmSession:
    """Open a bus session. Changes no drive state."""
    cfg = cfg or AmSessionConfig()
    _chk(cfg.cycle_us >= 100, AmStatus.AM_ERR_PARAM, "cycle_us must be at least 100")
    _chk(cfg.bus in (AmBus.ETHERCAT, AmBus.CANFD), AmStatus.AM_ERR_PARAM, "bus must be ETHERCAT or CANFD")
    backend = _backend_for(cfg)
    backend.open(cfg)
    return AmSession(cfg, backend)


def am_deinit(s: Optional[AmSession]) -> None:
    """Disable every joint, stop the session, free resources. Safe on None."""
    if s is None or s.closed:
        return
    if s.started:
        for j in s.joints:
            j.enable_req = False
            j.quick_stop_req = False
        try:
            am_cycle(s)
        except AmError:
            pass
        am_stop(s)
    s.backend.close()
    s.closed = True


def am_scan(s: AmSession) -> List[AmDeviceInfo]:
    """List the devices on the bus. Reads identity only and changes nothing. [NRT]"""
    return s.backend.scan()


def am_open_joint(s: AmSession, address: int) -> AmJoint:
    """Attach to one joint (address from am_scan; simulator: 0..n-1). Reads identity, transparency
    and scaling. Raises AM_ERR_VERSION on an unsupported OD major. Call before am_start(). [NRT]"""
    for j in s.joints:
        if j.address == address:
            return j
    jb = s.backend.open_joint(address)
    ident = jb.identity()
    major = (ident.od_version >> 24) & 0xFF
    _chk(major == OD_MAJOR_SUPPORTED or (OD_MAJOR_SUPPORTED > 0 and major == OD_MAJOR_SUPPORTED - 1),
         AmStatus.AM_ERR_VERSION, "object dictionary major %d not supported by this SDK" % major)
    j = AmJoint(s, address, jb)
    s.joints.append(j)
    return j


def am_start(s: AmSession) -> None:
    """Start cyclic exchange. Runs a few zero-command cycles so the drives report their state."""
    if s.started:
        return
    s.backend.start()
    s.started = True
    for _ in range(5):
        am_cycle(s)


def am_stop(s: AmSession) -> None:
    """Stop cyclic exchange: drives see comms loss, go to zero torque and disarm."""
    if not s.started:
        return
    s.backend.stop()
    for j in s.joints:
        j.enable_req = False
        j.quick_stop_req = False
        _dispatch(s, j)
    s.started = False


def _dispatch(s: AmSession, j: AmJoint):
    for kind, code, t in j.jb.pop_events():
        if s.cfg.event_cb is not None:
            s.cfg.event_cb(AmEvent(AmEventKind(kind), j, code, t))


def _controlword(j: AmJoint) -> int:
    sw = j.tlm.statusword if j.tlm is not None else 0
    st = codec.am_codec_decode_state(sw)
    if j.reset_req:
        j.reset_req = False
        return codec.CW_FAULT_RESET
    if j.quick_stop_req:
        return codec.CW_QUICK_STOP
    if j.enable_req:
        return codec.am_codec_next_controlword(sw, True)
    if st in (AmDriveState.READY_TO_SWITCH_ON, AmDriveState.SWITCHED_ON, AmDriveState.OPERATION_ENABLED):
        return codec.CW_SHUTDOWN
    return codec.CW_DISABLE_VOLTAGE


def am_cycle(s: AmSession) -> int:
    """Run one cycle: send every joint's latest command, receive feedback. Returns the cycle time in ns."""
    _chk(not s.closed, AmStatus.AM_ERR_STATE, "session closed")
    _chk(s.started, AmStatus.AM_ERR_STATE, "session not started (call am_start)")
    s.cycle_idx += 1
    s.t_ns = s.cycle_idx * s.cfg.cycle_us * 1000
    items = [(j.jb, _controlword(j), j.mode, j.cmd, j.cmd_fresh) for j in s.joints]
    tlms = s.backend.cycle(items)
    for j, t in zip(s.joints, tlms):
        j.cmd_fresh = False
        if t is None:                  # no feedback this cycle: am_read_telemetry() turns stale
            continue
        j.tlm, j.tlm_cycle = t, s.cycle_idx
        # after STO, a fault, a quick stop or any loss of Operation enabled the host must enable again
        if t.sto_active or t.state in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE,
                                       AmDriveState.QUICK_STOP_ACTIVE):
            j.enable_req = False
        if t.state == AmDriveState.SWITCH_ON_DISABLED and j.quick_stop_req:
            j.quick_stop_req = False
    for j in s.joints:
        _dispatch(s, j)
    return s.t_ns


# ---------------------------------------------------------------- identity
def am_get_identity(j: AmJoint) -> AmIdentity:
    """Identity block (0x1018, 0x2000, 0x2100). [RT]"""
    ident = j.jb.identity()
    return ident


def am_get_transparency(j: AmJoint) -> AmTransparency:
    """Rating and transparency data (0x6076, 0x2D00), SI units, NaN where not available. [RT]"""
    return copy.copy(j.transparency)


# ---------------------------------------------------------------- mode and state
def am_set_mode(j: AmJoint, mode: AmMode) -> None:
    """Select the mode used from the next cycle and reset the command. AmMode.CURRENT needs the
    capability bit and 0x2A11:3 = 1 (AM_ERR_NOT_SUPPORTED otherwise) and entry before Operation
    enabled (AM_ERR_STATE otherwise). [NRT]"""
    try:
        mode = AmMode(mode)
    except ValueError:
        raise AmError(AmStatus.AM_ERR_PARAM, "unknown mode")
    _chk(mode != AmMode.NONE, AmStatus.AM_ERR_PARAM, "choose a mode")
    if mode == AmMode.CURRENT:
        _chk(bool(j.identity.capabilities & AM_CAP_MODE_CURRENT), AmStatus.AM_ERR_NOT_SUPPORTED,
             "current mode not available on this unit")
        enabled = int.from_bytes(j.jb.sdo_read(0x2A11, 3), "little")
        _chk(bool(enabled), AmStatus.AM_ERR_NOT_SUPPORTED,
             "current mode is off by default: am_sdo_write(j, 0x2A11, 3, b'\\x01') enables it")
        _chk(j.state != AmDriveState.OPERATION_ENABLED, AmStatus.AM_ERR_STATE,
             "enter current mode before Operation enabled")
    j.mode = mode
    j.cmd = AmCommand(mode=mode)
    j.cmd_fresh = False


def _check_active(j: AmJoint):
    _chk(not j.session.closed, AmStatus.AM_ERR_STATE, "session closed")
    if j.tlm is not None:
        _chk(j.tlm.state not in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE), AmStatus.AM_ERR_FAULT,
             "drive in fault: " + ", ".join(am_fault_name(c) for c in j.jb.fault_info().codes))
    bus = j.session.cfg.bus
    ok = j.jb.authority_ok(bus) if hasattr(j.jb, "authority_ok") else j.jb.get_control_bus()[1] == int(bus)
    _chk(ok, AmStatus.AM_ERR_AUTHORITY, "this bus does not hold control authority")


def am_enable(j: AmJoint, timeout_ms: int = 1000) -> None:
    """Walk Shutdown, Switch on, Enable operation. Runs session cycles itself while it waits
    (all joints of the session advance with their latest commands)."""
    s = j.session
    _chk(s.started, AmStatus.AM_ERR_STATE, "session not started")
    _chk(j.mode != AmMode.NONE, AmStatus.AM_ERR_STATE, "select a mode with am_set_mode first")
    _check_active(j)
    t = j.tlm
    _chk(not (t and t.sto_active), AmStatus.AM_ERR_STO)
    j.enable_req = True
    j.quick_stop_req = False
    n = max(1, int(timeout_ms * 1000 / s.cfg.cycle_us))
    for _ in range(n):
        am_cycle(s)
        t = j.tlm
        if t is None:
            continue
        if t.state == AmDriveState.OPERATION_ENABLED:
            return
        if t.sto_active:
            j.enable_req = False
            raise AmError(AmStatus.AM_ERR_STO)
        if t.state in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE):
            j.enable_req = False
            raise AmError(AmStatus.AM_ERR_FAULT)
    j.enable_req = False
    if j.tlm is None:
        raise AmError(AmStatus.AM_ERR_BUS, "no feedback from the joint")
    if j.tlm.ref_state != AmRefState.VALID:
        raise AmError(AmStatus.AM_ERR_REFERENCE)
    raise AmError(AmStatus.AM_ERR_TIMEOUT)


def am_disable(j: AmJoint) -> None:
    """Disable operation: torque off, joint coasts. Lands in Ready to switch on."""
    j.enable_req = False
    j.quick_stop_req = False


def am_quick_stop(j: AmJoint) -> None:
    """Quick stop (torque ramp 0x6087 per 0x605A, then Switch on disabled). Not safety-rated."""
    j.quick_stop_req = True
    j.enable_req = False


# ---------------------------------------------------------------- commands
def _sanity(cmd: AmCommand):
    _finite(cmd.q_ref_rad, cmd.qd_ref_rad_s, cmd.tau_ff_Nm, cmd.kp_Nm_per_rad, cmd.kd_Nm_s_per_rad, cmd.iq_ref_A)
    _chk(abs(cmd.q_ref_rad) <= SANITY_Q and abs(cmd.qd_ref_rad_s) <= SANITY_RATE and
         abs(cmd.tau_ff_Nm) <= SANITY_TORQUE_NM and abs(cmd.iq_ref_A) <= SANITY_TORQUE_NM,
         AmStatus.AM_ERR_PARAM, "value outside the SDK sanity range")
    _chk(cmd.kp_Nm_per_rad >= 0 and cmd.kd_Nm_s_per_rad >= 0, AmStatus.AM_ERR_PARAM, "K and D must not be negative")


def am_set_command(j: AmJoint, cmd: AmCommand) -> None:
    """Full cyclic command for the next am_cycle(); cmd.mode must equal the mode set with am_set_mode.
    SI units, output side. Raises AM_ERR_PARAM (NaN, sanity range), AM_ERR_LIMIT (K or D above the
    caps), AM_ERR_FAULT, AM_ERR_AUTHORITY or AM_ERR_STATE. The drive clamps to the effective limits
    and sets statusword bit 11. [RT]"""
    _check_active(j)
    _chk(cmd.mode == j.mode, AmStatus.AM_ERR_STATE, "command mode differs from the mode set with am_set_mode")
    _sanity(cmd)
    if j.mode == AmMode.IMPEDANCE:
        lim = j.limits_cache
        _chk(cmd.kp_Nm_per_rad <= lim.kp_max_Nm_per_rad and cmd.kd_Nm_s_per_rad <= lim.kd_max_Nm_s_per_rad,
             AmStatus.AM_ERR_LIMIT,
             "K %.4g (max %.4g) or D %.4g (max %.4g) above the envelope" %
             (cmd.kp_Nm_per_rad, lim.kp_max_Nm_per_rad, cmd.kd_Nm_s_per_rad, lim.kd_max_Nm_s_per_rad))
    j.cmd = copy.copy(cmd)
    j.cmd_fresh = True


def am_set_impedance(j: AmJoint, imp: AmImpedance) -> None:
    """Impedance set-point (rad, rad/s, Nm/rad, Nm s/rad, Nm). Mode must be IMPEDANCE. [RT]"""
    _chk(j.mode == AmMode.IMPEDANCE, AmStatus.AM_ERR_STATE, "mode is not impedance")
    am_set_command(j, AmCommand(mode=AmMode.IMPEDANCE, q_ref_rad=imp.q_ref_rad, qd_ref_rad_s=imp.qd_ref_rad_s,
                                kp_Nm_per_rad=imp.kp_Nm_per_rad, kd_Nm_s_per_rad=imp.kd_Nm_s_per_rad,
                                tau_ff_Nm=imp.tau_ff_Nm))


def am_set_position(j: AmJoint, q_rad: float, qd_ff_rad_s: float = 0.0, tau_ff_Nm: float = 0.0) -> None:
    """CSP target (rad) with velocity (rad/s) and torque (Nm) feed-forward. Mode must be CSP. [RT]"""
    _chk(j.mode == AmMode.CSP, AmStatus.AM_ERR_STATE, "mode is not CSP")
    am_set_command(j, AmCommand(mode=AmMode.CSP, q_ref_rad=q_rad, qd_ref_rad_s=qd_ff_rad_s, tau_ff_Nm=tau_ff_Nm))


def am_set_velocity(j: AmJoint, qd_rad_s: float, tau_ff_Nm: float = 0.0) -> None:
    """CSV target (rad/s) with torque feed-forward (Nm). Mode must be CSV. [RT]"""
    _chk(j.mode == AmMode.CSV, AmStatus.AM_ERR_STATE, "mode is not CSV")
    am_set_command(j, AmCommand(mode=AmMode.CSV, qd_ref_rad_s=qd_rad_s, tau_ff_Nm=tau_ff_Nm))


def am_set_torque(j: AmJoint, tau_Nm: float) -> None:
    """CST target torque at the output (Nm, open loop, no torque sensor). Mode must be CST. [RT]"""
    _chk(j.mode == AmMode.CST, AmStatus.AM_ERR_STATE, "mode is not CST")
    am_set_command(j, AmCommand(mode=AmMode.CST, tau_ff_Nm=tau_Nm))


def am_set_current(j: AmJoint, iq_A: float) -> None:
    """q-axis current reference (A). Mode must be CURRENT. [RT]"""
    _chk(j.mode == AmMode.CURRENT, AmStatus.AM_ERR_STATE, "mode is not current")
    am_set_command(j, AmCommand(mode=AmMode.CURRENT, iq_ref_A=iq_A))


def am_read_telemetry(j: AmJoint) -> AmTelemetry:
    """Copy of the latest telemetry sample. Raises AM_ERR_STALE when it is older than
    stale_limit_cycles. [RT]"""
    _chk(j.tlm is not None, AmStatus.AM_ERR_STALE, "no telemetry yet (call am_start / am_cycle)")
    limit = j.session.cfg.stale_limit_cycles or 3
    _chk(j.session.cycle_idx - j.tlm_cycle <= limit, AmStatus.AM_ERR_STALE)
    return copy.copy(j.tlm)


# ---------------------------------------------------------------- faults
def am_get_faults(j: AmJoint) -> AmFaultInfo:
    """Current fault snapshot (0x603F, 0x2600, 0x2602, 0x2603, 0x2620). [NRT]"""
    return j.jb.fault_info()


def am_clear_faults(j: AmJoint, timeout_ms: int = 1000) -> None:
    """Clear latched faults (controlword bit 7 edge). The drive re-qualifies the reference; walk the
    enable sequence again afterwards."""
    s = j.session
    _chk(s.started, AmStatus.AM_ERR_STATE, "session not started")
    if j.tlm is not None and j.tlm.state not in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE):
        return
    j.reset_req = True
    n = max(2, int(timeout_ms * 1000 / s.cfg.cycle_us))
    for _ in range(n):
        am_cycle(s)
        if j.tlm.state not in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE):
            return
        j.reset_req = True if j.tlm.state == AmDriveState.FAULT and (s.cycle_idx % 2 == 0) else False
    raise AmError(AmStatus.AM_ERR_FAULT, "fault cause still present")


def am_get_fault_history(j: AmJoint) -> List[AmFaultRecord]:
    """Fault history (0x2601), newest first. [NRT]"""
    return j.jb.fault_history()


# am_fault_name and am_status_str are re-exported from types
def am_last_abort_code(j: AmJoint) -> int:
    """Last SDO abort code seen on this joint (CiA 301 numbering). [RT]"""
    return j.jb.last_abort_code()


# ---------------------------------------------------------------- limits, gains, config
def am_set_limits(j: AmJoint, lim: AmLimits) -> None:
    """Lower-only. A value above the AM ceiling raises AM_ERR_LIMIT and nothing is written."""
    _chk(j.state != AmDriveState.OPERATION_ENABLED, AmStatus.AM_ERR_STATE, "not while Operation enabled")
    j.jb.set_limits(lim)
    j.limits_cache = j.jb.get_limits()[0]


def am_get_limits(j: AmJoint) -> Tuple[AmLimits, AmLimits]:
    """(effective, ceiling)"""
    return j.jb.get_limits()


def am_set_loop_gains(j: AmJoint, g: AmLoopGains) -> None:
    """CSP/CSV gains and filters inside the 0x2A50 envelope (AM_ERR_LIMIT otherwise, nothing written).
    Not while Operation enabled. [NRT]"""
    _chk(j.state != AmDriveState.OPERATION_ENABLED, AmStatus.AM_ERR_STATE, "not while Operation enabled")
    j.jb.set_loop_gains(g)


def am_get_loop_gains(j: AmJoint) -> AmLoopGains:
    """Current CSP/CSV gains and filters. [NRT]"""
    return j.jb.get_loop_gains()


def am_store_parameters(j: AmJoint) -> None:
    """Persist your parameters (0x1010:1). Factory data is untouched. [NRT]"""
    j.jb.store_parameters()


def am_restore_defaults(j: AmJoint) -> None:
    """Restore your parameters to defaults (0x1011:1). Factory data is untouched. [NRT]"""
    j.jb.restore_defaults()
    j.limits_cache = j.jb.get_limits()[0]


def am_set_user_label(j: AmJoint, label: str) -> None:
    """Store a label on the drive (0x2E00:1, up to 31 characters). [NRT]"""
    j.jb.set_user_label(label)


def am_get_control_bus(j: AmJoint) -> Tuple[AmBus, AmBus]:
    """(configured, effective) control authority (0x2901:1, 0x2901:3). [NRT]"""
    c, e = j.jb.get_control_bus()
    return AmBus(c), AmBus(e)


def am_set_control_bus(j: AmJoint, bus: AmBus) -> None:
    """Configure the control bus. Only in Switch on disabled or lower; effective after the next drive
    reset. [NRT]"""
    j.jb.set_control_bus(int(bus))


def am_sdo_read(j: AmJoint, index: int, sub: int) -> bytes:
    """Raw object read (little-endian bytes). Raises AM_ERR_SDO_ABORT; see am_last_abort_code(). [NRT]"""
    return j.jb.sdo_read(index, sub)


def am_sdo_write(j: AmJoint, index: int, sub: int, data: bytes) -> None:
    """Raw object write. Factory and AM-internal objects abort (0x08000020/22). [NRT]"""
    j.jb.sdo_write(index, sub, bytes(data))


def am_read_calibration(j: AmJoint) -> str:
    """The calibration record (am_cal.json) as text. [NRT]"""
    return j.jb.read_calibration()


def am_trace_arm(j: AmJoint, trigger_signal: int = 0, trigger_level: float = 0.0, length: int = 0,
                 pretrigger: int = 0) -> None:
    """Arm the 20 kHz trace buffer. Not available in SDK 0.x."""
    raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "the 20 kHz trace buffer is not available in SDK 0.x")


def am_trace_read(j: AmJoint):
    """Read a completed trace. Not available in SDK 0.x."""
    raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "the 20 kHz trace buffer is not available in SDK 0.x")
