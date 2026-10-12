# SPDX-License-Identifier: Apache-2.0
"""Types and constants mirroring c/include/am_api.h (API 0.2.0)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, List, Optional

API_VERSION = (0, 2, 0)
OD_MAJOR_SUPPORTED = 0


def am_version_pack(maj: int, mn: int, pat: int) -> int:
    return ((maj & 0xFF) << 24) | ((mn & 0xFF) << 16) | (pat & 0xFFFF)


class AmStatus(IntEnum):
    AM_OK = 0
    AM_ERR_PARAM = -1
    AM_ERR_STATE = -2
    AM_ERR_TIMEOUT = -3
    AM_ERR_BUS = -4
    AM_ERR_NO_DEVICE = -5
    AM_ERR_SDO_ABORT = -6
    AM_ERR_NOT_SUPPORTED = -7
    AM_ERR_LIMIT = -8
    AM_ERR_AUTHORITY = -9
    AM_ERR_FAULT = -10
    AM_ERR_VERSION = -11
    AM_ERR_IMAGE = -12
    AM_ERR_BUSY = -13
    AM_ERR_NO_MEMORY = -14
    AM_ERR_STALE = -15
    AM_ERR_STO = -16
    AM_ERR_REFERENCE = -17
    AM_ERR_INTERNAL = -99


_STATUS_TEXT = {
    AmStatus.AM_OK: "ok",
    AmStatus.AM_ERR_PARAM: "invalid parameter",
    AmStatus.AM_ERR_STATE: "call not allowed in the current drive or session state",
    AmStatus.AM_ERR_TIMEOUT: "timeout",
    AmStatus.AM_ERR_BUS: "bus error",
    AmStatus.AM_ERR_NO_DEVICE: "no such device",
    AmStatus.AM_ERR_SDO_ABORT: "SDO abort",
    AmStatus.AM_ERR_NOT_SUPPORTED: "not supported on this unit, bus or firmware",
    AmStatus.AM_ERR_LIMIT: "value outside the safe envelope (not clamped, nothing written)",
    AmStatus.AM_ERR_AUTHORITY: "this bus does not hold control authority",
    AmStatus.AM_ERR_FAULT: "drive is in fault",
    AmStatus.AM_ERR_VERSION: "incompatible object dictionary or SDK version",
    AmStatus.AM_ERR_IMAGE: "firmware image rejected",
    AmStatus.AM_ERR_BUSY: "another operation is in progress on the joint",
    AmStatus.AM_ERR_NO_MEMORY: "out of memory",
    AmStatus.AM_ERR_STALE: "telemetry stale",
    AmStatus.AM_ERR_STO: "STO active",
    AmStatus.AM_ERR_REFERENCE: "absolute output reference not valid",
    AmStatus.AM_ERR_INTERNAL: "internal error",
}


def am_status_str(st: int) -> str:
    try:
        return _STATUS_TEXT[AmStatus(st)]
    except ValueError:
        return "unknown status"


class AmError(Exception):
    """Raised for every non-OK am_status. `.status` is the C status code."""

    def __init__(self, status: AmStatus, detail: str = ""):
        self.status = AmStatus(status)
        self.detail = detail
        msg = "%s (%d): %s" % (self.status.name, int(self.status), am_status_str(self.status))
        if detail:
            msg += ": " + detail
        super().__init__(msg)


class AmBus(IntEnum):
    NONE = 0
    ETHERCAT = 1
    CANFD = 2


class AmMode(IntEnum):
    NONE = 0
    CSP = 8
    CSV = 9
    CST = 10
    IMPEDANCE = -1
    CURRENT = -2


class AmDriveState(IntEnum):
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7


class AmSku(IntEnum):
    UNKNOWN = 0
    S_D = 1
    S_S = 2
    M = 3
    H_F = 4
    H_T = 5
    H_X = 6


class AmRefState(IntEnum):
    INVALID = 0
    CANDIDATE = 1
    VALID = 2


class AmDataBasis(IntEnum):
    NOT_AVAILABLE = 0
    DESIGN_TARGET = 1   # design value, not yet measured
    MEASURED_TYPE = 2
    MEASURED_UNIT = 3


class AmEventKind(IntEnum):
    FAULT = 1
    FAULT_CLEARED = 2
    WARNING = 3
    STO_CHANGED = 4
    LINK_LOST = 5
    LINK_RESTORED = 6
    STATE_CHANGED = 7


# capability bits (0x2000:8)
AM_CAP_OUTPUT_ABS_ENCODER = 1 << 0
AM_CAP_WINDING_SENSOR = 1 << 1
AM_CAP_ETHERCAT = 1 << 2
AM_CAP_CANFD_CONTROL = 1 << 3
AM_CAP_STO = 1 << 4
AM_CAP_BRAKE = 1 << 5
AM_CAP_MODE_CURRENT = 1 << 6
AM_CAP_TRACE_BUFFER = 1 << 7
AM_CAP_BUS_FW_UPDATE = 1 << 8

# sync flags (0x2800:3)
AM_SYNC_DC_LOCKED = 1 << 0
AM_SYNC_TIMESTAMP_VALID = 1 << 1
AM_SYNC_CAN_LOCKED = 1 << 2

# safety I/O bits (0x2620:1)
AM_SIO_STO1 = 1 << 0
AM_SIO_STO2 = 1 << 1
AM_SIO_GATE_SUPPLY_PRESENT = 1 << 2
AM_SIO_ENABLE_READBACK = 1 << 3
AM_SIO_DISCREPANCY = 1 << 4

# warning word bits (0x2805:2)
AM_WARN_THERMAL_DERATE = 1 << 0
AM_WARN_PEAK_BUDGET_EMPTY = 1 << 1
AM_WARN_SPEED_LIMIT = 1 << 2

# statusword bits beyond CiA 402 (0x6041), as in am_api.h
AM_SW_REMOTE = 1 << 9           # this channel holds control authority
AM_SW_INTERNAL_LIMIT = 1 << 11
AM_SW_REF_VALID = 1 << 14       # absolute output reference valid
AM_SW_STO = 1 << 15             # STO active


@dataclass
class AmIdentity:
    vendor_id: int = 0
    sku: AmSku = AmSku.UNKNOWN
    revision: int = 0
    serial: int = 0
    uid: bytes = b"\0" * 12
    od_version: int = 0
    capabilities: int = 0
    cal_record_version: int = 0
    cal_record_crc32: int = 0
    ratio_num: int = 1
    ratio_den: int = 1
    fw_main: int = 0
    fw_motor: int = 0
    fw_boot: int = 0
    device_name: str = ""
    user_label: str = ""


@dataclass
class AmTransparency:
    rated_torque_Nm: float = math.nan
    peak_torque_2s_Nm: float = math.nan
    rated_speed_rad_s: float = math.nan
    no_load_speed_48V_rad_s: float = math.nan
    reflected_inertia_kgm2: float = math.nan
    backdrive_torque_Nm: float = math.nan
    kinetic_energy_no_load_J: float = math.nan
    stiffness_Nm_per_rad: float = math.nan
    backlash_rad: float = math.nan
    sto_reaction_time_s: float = math.nan
    collision_latency_s: float = math.nan
    basis: AmDataBasis = AmDataBasis.NOT_AVAILABLE


@dataclass
class AmCommand:
    mode: AmMode = AmMode.NONE
    q_ref_rad: float = 0.0
    qd_ref_rad_s: float = 0.0
    tau_ff_Nm: float = 0.0
    kp_Nm_per_rad: float = 0.0
    kd_Nm_s_per_rad: float = 0.0
    iq_ref_A: float = 0.0


@dataclass
class AmImpedance:
    q_ref_rad: float = 0.0
    qd_ref_rad_s: float = 0.0
    kp_Nm_per_rad: float = 0.0
    kd_Nm_s_per_rad: float = 0.0
    tau_ff_Nm: float = 0.0


@dataclass
class AmTelemetry:
    t_ns: int = 0
    sample_counter: int = 0
    sync_flags: int = 0
    state: AmDriveState = AmDriveState.NOT_READY_TO_SWITCH_ON
    mode: AmMode = AmMode.NONE
    q_rad: float = 0.0
    qd_rad_s: float = 0.0
    q_motor_counts: int = 0
    qd_motor_rad_s: float = 0.0
    q_out_encoder_rad: float = 0.0
    twist_rad: float = 0.0
    id_A: float = 0.0
    iq_A: float = 0.0
    id_ref_A: float = 0.0
    iq_ref_A: float = 0.0
    vd_V: float = 0.0
    vq_V: float = 0.0
    vbus_V: float = 0.0
    ibus_est_A: float = 0.0
    tau_est_Nm: float = 0.0
    t_winding_C: float = 0.0
    t_magnet_C: float = 0.0
    t_power_stage_C: float = 0.0
    t_board_C: float = 0.0
    fault_word: int = 0
    error_code: int = 0
    warning_word: int = 0
    safety_io: int = 0
    ref_state: AmRefState = AmRefState.INVALID
    sto_active: bool = False
    internal_limit: bool = False
    extended_valid: bool = True
    statusword: int = 0


@dataclass
class AmLimits:
    torque_pos_Nm: float = 0.0
    torque_neg_Nm: float = 0.0
    current_A: float = 0.0          # phase current at the published 2 s peak torque (simulator); the drive command limit on the datasheet is higher
    speed_rad_s: float = 0.0
    pos_min_rad: float = -math.inf
    pos_max_rad: float = math.inf
    kp_max_Nm_per_rad: float = 0.0
    kd_max_Nm_s_per_rad: float = 0.0
    comms_loss_ms: int = 100


@dataclass
class AmLoopGains:
    pos_kp_1_per_s: float = 0.0
    vel_kp_Nm_s_per_rad: float = 0.0
    vel_ki_Nm_per_rad: float = 0.0
    vel_filter_hz: float = 300.0
    torque_cmd_filter_hz: float = 0.0
    notch_hz: float = 0.0
    notch_q: float = 1.0


@dataclass
class AmFaultInfo:
    error_code: int = 0
    fault_word: int = 0
    first_fault: int = 0
    warning_word: int = 0
    start_inhibit: int = 0
    safety_io: int = 0
    latched: bool = False
    codes: List[int] = field(default_factory=list)


@dataclass
class AmFaultRecord:
    code: int = 0
    drive_state: int = 0
    t_ns: int = 0
    vbus_V: float = 0.0
    iq_A: float = 0.0
    t_winding_C: float = 0.0
    t_power_stage_C: float = 0.0


@dataclass
class AmDeviceInfo:
    bus: AmBus = AmBus.NONE
    address: int = 0
    vendor_id: int = 0
    product_code: int = 0
    revision: int = 0
    serial: int = 0
    is_am_unit: bool = True
    od_compatible: bool = True
    has_control_authority: bool = True


@dataclass
class AmEvent:
    kind: AmEventKind
    joint: object = None
    code: int = 0
    t_ns: int = 0


@dataclass
class AmSessionConfig:
    """Session configuration. `interface` selects the backend:

    "sim" or "sim:M,H-F"                 built-in simulator, one joint per listed unit (addresses 0..n-1)
    "ecat:<nic>"                         EtherCAT through pysoem (pip install "am-sdk[ecat]")
    "can:<interface>:<channel>[@<nominal>/<data>]"
                                         CAN-FD through python-can (pip install "am-sdk[can]"),
                                         for example "can:socketcan:can0" or "can:pcan:PCAN_USBBUS1@1000000/5000000"
    """
    bus: AmBus = AmBus.ETHERCAT
    interface: str = "sim"
    cycle_us: int = 1000
    sdo_timeout_ms: int = 100
    stale_limit_cycles: int = 3
    canfd_nominal_bps: int = 0
    canfd_data_bps: int = 0
    canfd_compact: bool = False
    telemetry_extended: bool = True
    event_cb: Optional[Callable[[AmEvent], None]] = None
    log_cb: Optional[Callable[[int, str], None]] = None
    # sim backend options
    sim_units: Optional[List[str]] = None          # e.g. ["M", "M"]; overrides the interface string
    sim_options: Optional[List[dict]] = None       # per-unit overrides, see backends/sim.py SimOptions
    backend: Optional[object] = None               # a Backend instance (your own transport)


# fault codes (family_code, FAULT_CODES.json); bit n of the fault word = code 0x1000 + n
FAULT_NAMES = {
    0x1000: "DRV_NFAULT", 0x1001: "OVERCURRENT", 0x1002: "CUR_SAT", 0x1003: "CUR_IMPLAUS",
    0x1004: "CUR_KCL", 0x1005: "CUR_INVALID", 0x1006: "POS_INVALID", 0x1007: "POS_STALE",
    0x1008: "VBUS", 0x1009: "ISR_OVERRUN", 0x100A: "DRV_READBACK", 0x100B: "HOST_STALE",
    0x100C: "CLOCK_ORDER", 0x100D: "SEQUENCE", 0x100E: "RAIL", 0x100F: "NOCAL",
    0x1010: "PT1000", 0x1011: "INPUT_CURRENT",
    0x2000: "MOTOR_OVERTEMP_WINDING", 0x2001: "MAGNET_OVERTEMP_CEILING",
    0x2002: "POWERSTAGE_OVERTEMP", 0x2003: "TEMP_MODEL_INVALID",
    0x2100: "KT_RECORD_INVALID", 0x2200: "CONFIG_CRC_FAIL", 0x2201: "IMAGE_INCOMPATIBLE",
    0x2300: "ENC_CROSSCHECK_MISMATCH", 0x2301: "OUTPUT_ENCODER_LOST", 0x2302: "REFERENCE_LOST",
    0x3000: "COMMS_LOST_BUS", 0x3001: "CONTROL_AUTHORITY_CONFLICT", 0x3002: "CANFD_CONFIG_INVALID",
    0x4000: "SERVICE_WRITE_DENIED_ARMED",
}
FAULT_VBUS = 0x1008
FAULT_MOTOR_OVERTEMP = 0x2000
FAULT_COMMS_LOST = 0x3000
FAULT_HOST_STALE = 0x100B


def am_fault_name(code: int) -> str:
    return FAULT_NAMES.get(code, "UNKNOWN_0x%04X" % code)
