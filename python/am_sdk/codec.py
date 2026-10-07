"""L1 codec: pure functions, no I/O. Same names and byte layouts as the C codec (am_codec_*).

Host side (what a controller needs):
    am_codec_init, am_codec_pack_rx1600, am_codec_pack_rx1601, am_codec_unpack_tx1a00,
    am_codec_unpack_tx1a01, am_codec_pack_cfd_slot, am_codec_unpack_cfd_fb,
    am_codec_decode_state, am_codec_next_controlword.

Drive side (the inverse, used by the simulated nodes and by tests):
    drive_unpack_rx1600, drive_unpack_rx1601, drive_pack_tx1a00, drive_pack_tx1a01,
    drive_unpack_cfd_slot, drive_pack_cfd_fb.

All PDOs are little-endian. Byte layouts are in docs/API.md section 2.4 and 3.4.
A value that does not fit its wire field raises AmError(AM_ERR_PARAM); nothing is clamped.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

from .am_types import (AmCommand, AmDriveState, AmError, AmMode, AmRefState, AmStatus, AmTelemetry)

# ------------------------------------------------------------------ CiA 402 words
CW_DISABLE_VOLTAGE = 0x0000
CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_QUICK_STOP = 0x0002
CW_FAULT_RESET = 0x0080

SW_READY = 1 << 0
SW_SWITCHED_ON = 1 << 1
SW_OP_ENABLED = 1 << 2
SW_FAULT = 1 << 3
SW_VOLTAGE = 1 << 4
SW_QUICK_STOP_NOT = 1 << 5     # 0 = quick stop active
SW_SWITCH_ON_DISABLED = 1 << 6
SW_WARNING = 1 << 7
SW_REMOTE = 1 << 9
SW_INTERNAL_LIMIT = 1 << 11
SW_REF_VALID = 1 << 14         # AM: absolute output reference valid
SW_STO = 1 << 15               # AM: STO active

# statusword bit patterns per state (CiA 402)
STATE_TO_SW = {
    AmDriveState.NOT_READY_TO_SWITCH_ON: 0x0000,
    AmDriveState.SWITCH_ON_DISABLED: 0x0040,
    AmDriveState.READY_TO_SWITCH_ON: 0x0021,
    AmDriveState.SWITCHED_ON: 0x0023,
    AmDriveState.OPERATION_ENABLED: 0x0027,
    AmDriveState.QUICK_STOP_ACTIVE: 0x0007,
    AmDriveState.FAULT_REACTION_ACTIVE: 0x000F,
    AmDriveState.FAULT: 0x0008,
}

# ------------------------------------------------------------------ sizes
AM_PDO_RX_1600_LEN = 20
AM_PDO_RX_1601_LEN = 28
AM_PDO_TX_1A00_LEN = 24
AM_PDO_TX_1A01_LEN = 60
AM_CFD_CMD_SLOT_LEN = 16
AM_CFD_GROUP_LEN = 64
AM_CFD_FB_LEN = 16
AM_CFD_FB_EXT_LEN = 32

# compact control byte
AM_CC_ENABLE = 1 << 0
AM_CC_QUICK_STOP = 1 << 1
AM_CC_FAULT_RESET = 1 << 2
AM_CC_ZERO_TORQUE = 1 << 3
AM_CC_MODE_SHIFT = 4

# compact mode codes (control bits 4..7, feedback status bits 8..11)
_MODE_TO_CC = {AmMode.NONE: 0, AmMode.CST: 1, AmMode.CSV: 2, AmMode.CSP: 3, AmMode.IMPEDANCE: 4, AmMode.CURRENT: 5}
_CC_TO_MODE = {v: k for k, v in _MODE_TO_CC.items()}

# compact feedback status bits
CFS_STATE_MASK = 0x7
CFS_WARNING = 1 << 3
CFS_STO = 1 << 4
CFS_REF_VALID = 1 << 5
CFS_INTERNAL_LIMIT = 1 << 6
CFS_AUTHORITY = 1 << 7
CFS_MODE_SHIFT = 8
CFS_TIME_VALID = 1 << 12
CFS_FAULT = 1 << 13

_RX1600 = struct.Struct("<Hbxiihhi")
_RX1601 = struct.Struct("<HbxiiiIIi")
_TX1A00 = struct.Struct("<HbxiihHIBBh")
_TX1A01 = struct.Struct("<QHHiiHHihhhhhhHhhhhhiHxx")
_CFD_SLOT = struct.Struct("<ihhHHBBh")
_CFD_FB = struct.Struct("<ihhhHBBH")
_CFD_EXT = struct.Struct("<IiHhhH")

assert _RX1600.size == AM_PDO_RX_1600_LEN and _RX1601.size == AM_PDO_RX_1601_LEN
assert _TX1A00.size == AM_PDO_TX_1A00_LEN and _TX1A01.size == AM_PDO_TX_1A01_LEN
assert _CFD_SLOT.size == AM_CFD_CMD_SLOT_LEN and _CFD_FB.size == AM_CFD_FB_LEN
assert _CFD_FB.size + _CFD_EXT.size == AM_CFD_FB_EXT_LEN

TWO_PI = 2.0 * math.pi


@dataclass
class AmCodec:
    """Per-joint scaling (am_codec in C). Build it with am_codec_init()."""
    rated_torque_Nm: float = 1.0
    rated_current_A: float = 1.0
    compact_k_scale: float = 0.02
    compact_d_scale: float = 0.001
    compact_slot: int = 0


def am_codec_init(rated_torque_mNm: int, rated_current_mA: int, compact_k_mNm_per_rad_lsb: int = 20,
                  compact_d_mNm_s_per_rad_lsb: int = 1, compact_slot: int = 0) -> AmCodec:
    if rated_torque_mNm <= 0 or rated_current_mA <= 0 or compact_k_mNm_per_rad_lsb <= 0 \
            or compact_d_mNm_s_per_rad_lsb <= 0 or not 0 <= compact_slot <= 3:
        raise AmError(AmStatus.AM_ERR_PARAM, "codec scaling out of range")
    return AmCodec(rated_torque_mNm * 1e-3, rated_current_mA * 1e-3, compact_k_mNm_per_rad_lsb * 1e-3,
                   compact_d_mNm_s_per_rad_lsb * 1e-3, compact_slot)


# ------------------------------------------------------------------ helpers
def _q(v: float, scale: float, lo: int, hi: int, what: str) -> int:
    """Scale and round half away from zero (as C lround). Raises AM_ERR_PARAM when it does not fit."""
    if v is None or not math.isfinite(v):
        raise AmError(AmStatus.AM_ERR_PARAM, "%s is not finite" % what)
    x = v * scale
    r = math.floor(abs(x) + 0.5)
    r = int(r if x >= 0 else -r)
    if r < lo or r > hi:
        raise AmError(AmStatus.AM_ERR_PARAM, "%s %g does not fit its wire field" % (what, v))
    return r


I16 = (-32768, 32767)
U16 = (0, 65535)
I32 = (-2147483648, 2147483647)
U32 = (0, 4294967295)


def _mode_cc(mode) -> int:
    try:
        return _MODE_TO_CC[AmMode(mode)]
    except (ValueError, KeyError):
        raise AmError(AmStatus.AM_ERR_PARAM, "unknown mode")


# ------------------------------------------------------------------ state machine helpers
def am_codec_decode_state(sw: int) -> AmDriveState:
    """Decode the CiA 402 state from a statusword (CiA 402 masks 0x4F / 0x6F)."""
    if (sw & 0x4F) == 0x00:
        return AmDriveState.NOT_READY_TO_SWITCH_ON
    if (sw & 0x4F) == 0x40:
        return AmDriveState.SWITCH_ON_DISABLED
    if (sw & 0x6F) == 0x21:
        return AmDriveState.READY_TO_SWITCH_ON
    if (sw & 0x6F) == 0x23:
        return AmDriveState.SWITCHED_ON
    if (sw & 0x6F) == 0x27:
        return AmDriveState.OPERATION_ENABLED
    if (sw & 0x6F) == 0x07:
        return AmDriveState.QUICK_STOP_ACTIVE
    if (sw & 0x4F) == 0x0F:
        return AmDriveState.FAULT_REACTION_ACTIVE
    if (sw & 0x4F) == 0x08:
        return AmDriveState.FAULT
    return AmDriveState.NOT_READY_TO_SWITCH_ON


def am_codec_next_controlword(sw: int, enable: bool) -> int:
    """Next controlword for walking toward Operation enabled (enable) or Switch on disabled."""
    st = am_codec_decode_state(sw)
    if not enable:
        return CW_DISABLE_VOLTAGE
    if st == AmDriveState.SWITCH_ON_DISABLED:
        return CW_SHUTDOWN
    if st == AmDriveState.READY_TO_SWITCH_ON:
        return CW_SWITCH_ON
    if st in (AmDriveState.SWITCHED_ON, AmDriveState.OPERATION_ENABLED):
        return CW_ENABLE_OPERATION
    return CW_DISABLE_VOLTAGE


# ------------------------------------------------------------------ host side: EtherCAT PDOs
def am_codec_pack_rx1601(c: AmCodec, cw: int, cmd: AmCommand) -> bytes:
    """RxPDO 0x1601: controlword, mode, target position, target velocity, torque (mNm), K, D, iq (mA)."""
    if cmd.kp_Nm_per_rad < 0 or cmd.kd_Nm_s_per_rad < 0:
        raise AmError(AmStatus.AM_ERR_PARAM, "K and D must not be negative")
    _mode_cc(cmd.mode)
    return _RX1601.pack(cw & 0xFFFF, int(cmd.mode),
                        _q(cmd.q_ref_rad, 1e6, *I32, "q_ref"), _q(cmd.qd_ref_rad_s, 1e3, *I32, "qd_ref"),
                        _q(cmd.tau_ff_Nm, 1e3, *I32, "tau_ff"), _q(cmd.kp_Nm_per_rad, 1e3, *U32, "K"),
                        _q(cmd.kd_Nm_s_per_rad, 1e3, *U32, "D"), _q(cmd.iq_ref_A, 1e3, *I32, "iq_ref"))


def am_codec_pack_rx1600(c: AmCodec, cw: int, cmd: AmCommand) -> bytes:
    """RxPDO 0x1600 (CiA 402 generic). CSP, CSV and CST only; torque in per mille of the rated torque."""
    mode = AmMode(cmd.mode)
    if mode not in (AmMode.CSP, AmMode.CSV, AmMode.CST, AmMode.NONE):
        raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "RxPDO 0x1600 carries CSP, CSV and CST only; use 0x1601")
    pm = 1000.0 / c.rated_torque_Nm
    q = _q(cmd.q_ref_rad, 1e6, *I32, "q_ref") if mode == AmMode.CSP else 0
    v_target = _q(cmd.qd_ref_rad_s, 1e3, *I32, "qd_ref") if mode == AmMode.CSV else 0
    v_offset = _q(cmd.qd_ref_rad_s, 1e3, *I32, "qd_ref") if mode == AmMode.CSP else 0
    t_target = _q(cmd.tau_ff_Nm, pm, *I16, "tau") if mode == AmMode.CST else 0
    t_offset = _q(cmd.tau_ff_Nm, pm, *I16, "tau_ff") if mode in (AmMode.CSP, AmMode.CSV) else 0
    return _RX1600.pack(cw & 0xFFFF, int(mode), q, v_target, t_target, t_offset, v_offset)


def am_codec_unpack_tx1a00(c: AmCodec, data: bytes, t: Optional[AmTelemetry] = None) -> AmTelemetry:
    """TxPDO 0x1A00 into the status part of the telemetry."""
    if len(data) < AM_PDO_TX_1A00_LEN:
        raise AmError(AmStatus.AM_ERR_PARAM, "TxPDO 0x1A00 needs 24 bytes")
    t = t if t is not None else AmTelemetry()
    sw, mode, q, qd, tq, err, fw, sio, ref, tw = _TX1A00.unpack_from(data)
    t.statusword = sw
    t.state = am_codec_decode_state(sw)
    t.sto_active = bool(sw & SW_STO)
    t.internal_limit = bool(sw & SW_INTERNAL_LIMIT)
    try:
        t.mode = AmMode(mode)
    except ValueError:
        t.mode = AmMode.NONE
    t.q_rad = q * 1e-6
    t.qd_rad_s = qd * 1e-3
    t.tau_est_Nm = tq * c.rated_torque_Nm * 1e-3
    t.error_code = err
    t.fault_word = fw
    t.safety_io = sio
    t.ref_state = AmRefState(ref) if ref in (0, 1, 2) else AmRefState.INVALID
    t.t_winding_C = tw * 0.1
    return t


def am_codec_unpack_tx1a01(c: AmCodec, data: bytes, t: Optional[AmTelemetry] = None) -> AmTelemetry:
    """TxPDO 0x1A01 into the estimator part of the telemetry."""
    if len(data) < AM_PDO_TX_1A01_LEN:
        raise AmError(AmStatus.AM_ERR_PARAM, "TxPDO 0x1A01 needs 60 bytes")
    t = t if t is not None else AmTelemetry()
    (t_ns, cnt, sync, mpos, mvel, oang, _encst, twist, i_d, i_q, idr, iqr, vd, vq, vbus, ibus,
     tw, tm, tps, tb, tau, warn) = _TX1A01.unpack_from(data)
    t.t_ns, t.sample_counter, t.sync_flags = t_ns, cnt, sync
    t.q_motor_counts = mpos
    t.qd_motor_rad_s = mvel * 1e-3
    t.q_out_encoder_rad = oang * TWO_PI / 65536.0
    t.twist_rad = twist * 1e-6
    t.id_A, t.iq_A, t.id_ref_A, t.iq_ref_A = i_d * 0.01, i_q * 0.01, idr * 0.01, iqr * 0.01
    t.vd_V, t.vq_V = vd * 0.01, vq * 0.01
    t.vbus_V = vbus * 1e-3
    t.ibus_est_A = ibus * 0.01
    t.t_winding_C, t.t_magnet_C, t.t_power_stage_C, t.t_board_C = tw * 0.1, tm * 0.1, tps * 0.1, tb * 0.1
    t.tau_est_Nm = tau * 1e-3
    t.warning_word = warn
    t.extended_valid = True
    return t


# ------------------------------------------------------------------ host side: CAN-FD compact profile
def am_codec_pack_cfd_slot(c: AmCodec, control: int, counter: int, cmd: AmCommand) -> bytes:
    """One 16 B joint slot of the CAN-FD group command frame. K and D are sent only in impedance mode."""
    mode = AmMode(cmd.mode)
    if cmd.kp_Nm_per_rad < 0 or cmd.kd_Nm_s_per_rad < 0:
        raise AmError(AmStatus.AM_ERR_PARAM, "K and D must not be negative")
    if mode == AmMode.CURRENT:
        third = _q(cmd.iq_ref_A, 100.0, *I16, "iq_ref")
    else:
        third = _q(cmd.tau_ff_Nm, 100.0, *I16, "tau_ff")
    imp = mode == AmMode.IMPEDANCE
    k = _q(cmd.kp_Nm_per_rad, 1.0 / c.compact_k_scale, *U16, "K") if imp else 0
    d = _q(cmd.kd_Nm_s_per_rad, 1.0 / c.compact_d_scale, *U16, "D") if imp else 0
    return _CFD_SLOT.pack(_q(cmd.q_ref_rad, 1e6, *I32, "q_ref"), _q(cmd.qd_ref_rad_s, 100.0, *I16, "qd_ref"),
                          third, k, d, control & 0xFF, counter & 0xFF, 0)


def am_codec_unpack_cfd_fb(c: AmCodec, data: bytes, t: Optional[AmTelemetry] = None) -> Tuple[AmTelemetry, int]:
    """Compact (16 B) or estimator-lite (32 B) feedback. Returns (telemetry, echoed counter)."""
    if len(data) not in (AM_CFD_FB_LEN, AM_CFD_FB_EXT_LEN):
        raise AmError(AmStatus.AM_ERR_PARAM, "CAN-FD feedback is 16 or 32 bytes")
    t = t if t is not None else AmTelemetry()
    q, qd, tau, iq, st, cnt, tw, vb = _CFD_FB.unpack_from(data)
    state = AmDriveState(st & CFS_STATE_MASK)
    sw = STATE_TO_SW[state]
    if state in (AmDriveState.READY_TO_SWITCH_ON, AmDriveState.SWITCHED_ON, AmDriveState.OPERATION_ENABLED,
                 AmDriveState.QUICK_STOP_ACTIVE):
        sw |= SW_VOLTAGE
    if st & CFS_WARNING:
        sw |= SW_WARNING
    if st & CFS_AUTHORITY:
        sw |= SW_REMOTE
    if st & CFS_INTERNAL_LIMIT:
        sw |= SW_INTERNAL_LIMIT
    if st & CFS_REF_VALID:
        sw |= SW_REF_VALID
    if st & CFS_STO:
        sw |= SW_STO
    t.statusword = sw
    t.state = state
    t.sto_active = bool(st & CFS_STO)
    t.internal_limit = bool(st & CFS_INTERNAL_LIMIT)
    t.ref_state = AmRefState.VALID if st & CFS_REF_VALID else AmRefState.INVALID
    t.mode = _CC_TO_MODE.get((st >> CFS_MODE_SHIFT) & 0xF, AmMode.NONE)
    t.q_rad = q * 1e-6
    t.qd_rad_s = qd * 0.01
    t.tau_est_Nm = tau * 0.01
    t.iq_A = iq * 0.01
    t.t_winding_C = float(tw) - 40.0
    t.vbus_V = vb * 0.01
    if len(data) == AM_CFD_FB_EXT_LEN:
        ts, mpos, oang, i_d, tps, warn = _CFD_EXT.unpack_from(data, AM_CFD_FB_LEN)
        t.t_ns = ts
        t.q_motor_counts = mpos
        t.q_out_encoder_rad = oang * TWO_PI / 65536.0
        t.id_A = i_d * 0.01
        t.t_power_stage_C = tps * 0.1
        t.warning_word = warn
        t.extended_valid = True
    else:
        t.extended_valid = False
    return t, cnt


# ------------------------------------------------------------------ drive side (simulated nodes, tests)
def drive_unpack_rx1601(data: bytes) -> Tuple[int, AmCommand]:
    cw, mode, q, qd, tau, k, d, iq = _RX1601.unpack_from(data)
    try:
        m = AmMode(mode)
    except ValueError:
        m = AmMode.NONE
    return cw, AmCommand(mode=m, q_ref_rad=q * 1e-6, qd_ref_rad_s=qd * 1e-3, tau_ff_Nm=tau * 1e-3,
                         kp_Nm_per_rad=k * 1e-3, kd_Nm_s_per_rad=d * 1e-3, iq_ref_A=iq * 1e-3)


def drive_unpack_rx1600(c: AmCodec, data: bytes) -> Tuple[int, AmCommand]:
    cw, mode, q, vt, tt, to, vo = _RX1600.unpack_from(data)
    try:
        m = AmMode(mode)
    except ValueError:
        m = AmMode.NONE
    r = c.rated_torque_Nm * 1e-3
    if m == AmMode.CST:
        cmd = AmCommand(mode=m, tau_ff_Nm=tt * r)
    elif m == AmMode.CSV:
        cmd = AmCommand(mode=m, qd_ref_rad_s=vt * 1e-3, tau_ff_Nm=to * r)
    else:
        cmd = AmCommand(mode=m, q_ref_rad=q * 1e-6, qd_ref_rad_s=vo * 1e-3, tau_ff_Nm=to * r)
    return cw, cmd


def _sat(x: float, lo: int, hi: int) -> int:
    """Drive-side encoding of measured values: round half away from zero, saturate."""
    if not math.isfinite(x):
        return 0
    r = math.floor(abs(x) + 0.5)
    r = int(r if x >= 0 else -r)
    return lo if r < lo else hi if r > hi else r


def drive_pack_tx1a00(c: AmCodec, t: AmTelemetry) -> bytes:
    return _TX1A00.pack(t.statusword & 0xFFFF, int(t.mode), _sat(t.q_rad * 1e6, *I32), _sat(t.qd_rad_s * 1e3, *I32),
                        _sat(t.tau_est_Nm * 1e3 / c.rated_torque_Nm, *I16), t.error_code & 0xFFFF,
                        t.fault_word & 0xFFFFFFFF, t.safety_io & 0xFF, int(t.ref_state), _sat(t.t_winding_C * 10, *I16))


def drive_pack_tx1a01(t: AmTelemetry) -> bytes:
    oang = _sat(t.q_out_encoder_rad / TWO_PI * 65536.0, -2 ** 40, 2 ** 40) % 65536
    return _TX1A01.pack(t.t_ns & 0xFFFFFFFFFFFFFFFF, t.sample_counter & 0xFFFF, t.sync_flags & 0xFFFF,
                        _sat(t.q_motor_counts, *I32), _sat(t.qd_motor_rad_s * 1e3, *I32), oang,
                        int(t.ref_state) << 8, _sat(t.twist_rad * 1e6, *I32),
                        _sat(t.id_A * 100, *I16), _sat(t.iq_A * 100, *I16), _sat(t.id_ref_A * 100, *I16),
                        _sat(t.iq_ref_A * 100, *I16), _sat(t.vd_V * 100, *I16), _sat(t.vq_V * 100, *I16),
                        _sat(t.vbus_V * 1e3, *U16), _sat(t.ibus_est_A * 100, *I16),
                        _sat(t.t_winding_C * 10, *I16), _sat(t.t_magnet_C * 10, *I16),
                        _sat(t.t_power_stage_C * 10, *I16), _sat(t.t_board_C * 10, *I16),
                        _sat(t.tau_est_Nm * 1e3, *I32), t.warning_word & 0xFFFF)


def drive_unpack_cfd_slot(c: AmCodec, data: bytes) -> Tuple[int, int, AmCommand]:
    """Returns (control byte, counter, command). The mode comes from control bits 4..7 (0 = keep)."""
    q, qd, third, k, d, control, counter, _ = _CFD_SLOT.unpack_from(data)
    mode = _CC_TO_MODE.get((control >> AM_CC_MODE_SHIFT) & 0xF, AmMode.NONE)
    cmd = AmCommand(mode=mode, q_ref_rad=q * 1e-6, qd_ref_rad_s=qd * 0.01,
                    tau_ff_Nm=0.0 if mode == AmMode.CURRENT else third * 0.01,
                    iq_ref_A=third * 0.01 if mode == AmMode.CURRENT else 0.0,
                    kp_Nm_per_rad=k * c.compact_k_scale, kd_Nm_s_per_rad=d * c.compact_d_scale)
    return control, counter, cmd


def drive_pack_cfd_fb(t: AmTelemetry, counter: int, authority: bool, extended: bool) -> bytes:
    st = int(t.state) & CFS_STATE_MASK
    if t.statusword & SW_WARNING:
        st |= CFS_WARNING
    if t.sto_active:
        st |= CFS_STO
    if t.ref_state == AmRefState.VALID:
        st |= CFS_REF_VALID
    if t.internal_limit:
        st |= CFS_INTERNAL_LIMIT
    if authority:
        st |= CFS_AUTHORITY
    st |= _MODE_TO_CC.get(t.mode, 0) << CFS_MODE_SHIFT
    if t.sync_flags & 0x2:
        st |= CFS_TIME_VALID
    if t.state in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE):
        st |= CFS_FAULT
    out = _CFD_FB.pack(_sat(t.q_rad * 1e6, *I32), _sat(t.qd_rad_s * 100, *I16), _sat(t.tau_est_Nm * 100, *I16),
                       _sat(t.iq_A * 100, *I16), st, counter & 0xFF, _sat(t.t_winding_C + 40.0, 0, 255),
                       _sat(t.vbus_V * 100, *U16))
    if extended:
        oang = _sat(t.q_out_encoder_rad / TWO_PI * 65536.0, -2 ** 40, 2 ** 40) % 65536
        out += _CFD_EXT.pack(t.t_ns & 0xFFFFFFFF, _sat(t.q_motor_counts, *I32), oang, _sat(t.id_A * 100, *I16),
                             _sat(t.t_power_stage_C * 10, *I16), t.warning_word & 0xFFFF)
    return out


def compact_control(cw: int, mode: AmMode, enable: bool) -> int:
    """Host side: map the SDK's controlword request to the compact control byte."""
    ctl = _mode_cc(mode) << AM_CC_MODE_SHIFT
    if cw == CW_FAULT_RESET:
        ctl |= AM_CC_FAULT_RESET
    elif cw == CW_QUICK_STOP:
        ctl |= AM_CC_QUICK_STOP
    elif enable:
        ctl |= AM_CC_ENABLE
    return ctl
