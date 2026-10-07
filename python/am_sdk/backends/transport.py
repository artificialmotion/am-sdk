"""Bus backends built on a small Transport interface (the Python form of am_transport_ops in the C header).

A Transport moves bytes: one cyclic process-data exchange for all opened joints, and confirmed
object reads and writes (CoE SDO on EtherCAT, SDO on CAN-FD). TransportBackend turns that into the
full JointBackend API with the L1 codec, so EtherCAT, CAN-FD, the wire-level simulator and your own
master all behave the same above this line.

Per-joint process images (order = order of am_open_joint):
    EtherCAT, CAN-FD profile 0:  out RxPDO 0x1601 (28 B); in TxPDO 0x1A00 (24 B) + 0x1A01 (60 B) when
                                 telemetry_extended is set
    CAN-FD compact profile:      out one 16 B group-frame slot; in 16 B compact or 32 B estimator-lite feedback
"""
from __future__ import annotations

import copy
import math
import struct
import time
from typing import List, Optional, Sequence, Tuple

from .. import codec as C
from ..am_types import (AmBus, AmCommand, AmDataBasis, AmDeviceInfo, AmDriveState, AmError, AmEventKind, AmFaultInfo,
                        AmFaultRecord, AmIdentity, AmLimits, AmLoopGains, AmMode, AmSessionConfig,
                        AmSku, AmStatus, AmTelemetry, AmTransparency, AM_SYNC_CAN_LOCKED, AM_SYNC_TIMESTAMP_VALID,
                        OD_MAJOR_SUPPORTED)
from .base import Backend, CycleItem, JointBackend

ABORT_RANGE = (0x06090030, 0x06090031, 0x06090032, 0x06090036)
ABORT_STATE = 0x08000022
STORE_SIGNATURE = 0x65766173      # "save"
RESTORE_SIGNATURE = 0x64616F6C    # "load"
CAN_NODE_IDS = range(1, 64)


class SdoAbort(Exception):
    """Raised by a Transport when the device answers an object access with an abort code."""

    def __init__(self, code: int, text: str = ""):
        self.code = code
        super().__init__("SDO abort 0x%08X%s" % (code, (": " + text) if text else ""))


class Transport:
    """Implement this to run the SDK over your own master or adapter."""

    name = "transport"
    realtime = False          # True: TransportBackend paces cycles to cfg.cycle_us of wall-clock time

    def open(self, cfg: AmSessionConfig) -> None:
        pass

    def close(self) -> None:
        pass

    def probe(self) -> Optional[List[int]]:
        """Addresses present on the bus, or None to let the SDK probe with object reads."""
        return None

    def setup(self, addresses: List[int], profile: str, extended: bool) -> None:
        """Configure process data for these joints (in this order) before start()."""

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def pdo_exchange(self, out: List[bytes]) -> Tuple[List[Optional[bytes]], int]:
        """Send one cycle; return the input image per joint (None if it did not arrive) and the cycle time in ns."""
        raise NotImplementedError

    def sdo_read(self, address: int, index: int, sub: int) -> bytes:
        raise NotImplementedError

    def sdo_write(self, address: int, index: int, sub: int, data: bytes) -> None:
        raise NotImplementedError

    def file_read(self, address: int, name: str) -> bytes:
        raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "file transfer not available on this transport")


def _u(b: bytes, fmt: str):
    return struct.unpack(fmt, bytes(b)[:struct.calcsize(fmt)].ljust(struct.calcsize(fmt), b"\0"))[0]


class TransportJoint(JointBackend):
    def __init__(self, be: "TransportBackend", address: int):
        self.be, self.tp, self.address = be, be.transport, address
        self.last_abort = 0
        self.events: List[Tuple[int, int, int]] = []
        self.tlm: Optional[AmTelemetry] = None
        self.have_cmd = False
        self.neutral_q = 0.0
        self.last_mode = AmMode.NONE
        self.counter = 0
        self.echo = 0
        self._read_identity()

    # ---------------------------------------------------------------- object access
    def _rd(self, index: int, sub: int) -> bytes:
        try:
            return bytes(self.tp.sdo_read(self.address, index, sub))
        except SdoAbort as e:
            self.last_abort = e.code
            raise AmError(AmStatus.AM_ERR_SDO_ABORT, "0x%04X:%d abort 0x%08X" % (index, sub, e.code))

    def _wr(self, index: int, sub: int, data: bytes, limit_ctx: bool = False) -> None:
        try:
            self.tp.sdo_write(self.address, index, sub, bytes(data))
        except SdoAbort as e:
            self.last_abort = e.code
            if limit_ctx and e.code in ABORT_RANGE:
                raise AmError(AmStatus.AM_ERR_LIMIT, "0x%04X:%d abort 0x%08X" % (index, sub, e.code))
            if limit_ctx and e.code == ABORT_STATE:
                raise AmError(AmStatus.AM_ERR_STATE, "0x%04X:%d abort 0x%08X" % (index, sub, e.code))
            raise AmError(AmStatus.AM_ERR_SDO_ABORT, "0x%04X:%d abort 0x%08X" % (index, sub, e.code))

    def _rd_opt(self, index: int, sub: int, fmt: str, default):
        try:
            return _u(self._rd(index, sub), fmt)
        except AmError:
            return default

    # ---------------------------------------------------------------- identity
    def _read_identity(self):
        rd = lambda i, s, f="<I": _u(self._rd(i, s), f)
        cal = self._rd(0x2000, 9).ljust(8, b"\0")
        try:
            sku = AmSku(rd(0x1018, 2))
        except ValueError:
            sku = AmSku.UNKNOWN
        self.ident = AmIdentity(
            vendor_id=rd(0x1018, 1), sku=sku, revision=rd(0x1018, 3), serial=rd(0x1018, 4),
            uid=self._rd(0x2000, 5)[:12].ljust(12, b"\0"), od_version=rd(0x2000, 6), capabilities=rd(0x2000, 8),
            cal_record_version=_u(cal[:4], "<I"), cal_record_crc32=_u(cal[4:8], "<I"),
            ratio_num=rd(0x6091, 1), ratio_den=rd(0x6091, 2), fw_cm7=rd(0x2100, 1), fw_cm4=rd(0x2100, 2),
            fw_boot=rd(0x2100, 3), device_name=self._rd(0x1008, 0).decode(errors="replace").rstrip("\0"),
            user_label=self._rd(0x2E00, 1).decode(errors="replace").rstrip("\0"))
        rated_mNm = rd(0x6076, 0)
        rated_mA = rd(0x6075, 0)
        k_lsb = self._rd_opt(0x2A20, 1, "<H", 20)
        d_lsb = self._rd_opt(0x2A20, 2, "<H", 1)
        slot = self._rd_opt(0x2902, 3, "<B", 0)
        self.group_cob = self._rd_opt(0x2902, 2, "<I", 0x1C0)
        self.codec = C.am_codec_init(rated_mNm, rated_mA, k_lsb or 20, d_lsb or 1, slot)

        def na(v, scale):
            return math.nan if v == 0xFFFFFFFF else v * scale

        def tr(sub, fmt, scale=1.0):
            v = self._rd_opt(0x2D00, sub, fmt, None)
            if v is None:
                return math.nan
            return na(v, scale) if fmt == "<I" else float(v)

        basis = self._rd_opt(0x2D00, 12, "<B", 0)
        self.transp = AmTransparency(
            rated_torque_Nm=rated_mNm * 1e-3, peak_torque_2s_Nm=tr(2, "<I", 1e-3), rated_speed_rad_s=tr(3, "<I", 1e-3),
            no_load_speed_48V_rad_s=tr(4, "<I", 1e-3), reflected_inertia_kgm2=tr(5, "<f"),
            backdrive_torque_Nm=tr(6, "<I", 1e-3), kinetic_energy_no_load_J=tr(7, "<f"),
            stiffness_Nm_per_rad=tr(8, "<f"), backlash_rad=tr(9, "<I", 1e-6), sto_reaction_time_s=tr(10, "<I", 1e-6),
            collision_latency_s=tr(11, "<I", 1e-6),
            basis=AmDataBasis(basis) if basis in (0, 1, 2, 3) else AmDataBasis.NOT_AVAILABLE)

    def identity(self) -> AmIdentity:
        return copy.copy(self.ident)

    def transparency(self) -> AmTransparency:
        return copy.copy(self.transp)

    # ---------------------------------------------------------------- cyclic
    def pack(self, cw: int, mode: AmMode, cmd: AmCommand, fresh: bool) -> bytes:
        t = self.tlm
        if t is None or t.state != AmDriveState.OPERATION_ENABLED:
            self.have_cmd = False
            self.neutral_q = t.q_rad if t is not None else 0.0
        if mode != self.last_mode:
            self.have_cmd = False
            self.neutral_q = t.q_rad if t is not None else 0.0
            self.last_mode = mode
        if fresh:
            self.have_cmd = True
        if self.have_cmd:
            send = cmd
        elif mode == AmMode.CSP:
            send = AmCommand(mode=mode, q_ref_rad=self.neutral_q)
        else:
            send = AmCommand(mode=mode)
        if self.be.profile == "compact":
            st = t.state if t is not None else AmDriveState.NOT_READY_TO_SWITCH_ON
            enable = cw in (C.CW_SWITCH_ON, C.CW_ENABLE_OPERATION) or (
                cw == C.CW_SHUTDOWN and st == AmDriveState.SWITCH_ON_DISABLED)
            self.counter = (self.counter + 1) & 0xFF
            return C.am_codec_pack_cfd_slot(self.codec, C.compact_control(cw, mode, enable), self.counter, send)
        return C.am_codec_pack_rx1601(self.codec, cw, send)

    def unpack(self, data: Optional[bytes], t_ns: int) -> Optional[AmTelemetry]:
        if data is None:
            return None
        prev = self.tlm
        t = copy.copy(prev) if prev is not None else AmTelemetry()
        if self.be.profile == "compact":
            t, self.echo = C.am_codec_unpack_cfd_fb(self.codec, data, t)
            t.sync_flags = AM_SYNC_CAN_LOCKED | AM_SYNC_TIMESTAMP_VALID
            if not t.extended_valid:
                t.t_ns = t_ns
        else:
            C.am_codec_unpack_tx1a00(self.codec, data[:C.AM_PDO_TX_1A00_LEN], t)
            if len(data) >= C.AM_PDO_TX_1A00_LEN + C.AM_PDO_TX_1A01_LEN:
                C.am_codec_unpack_tx1a01(self.codec, data[C.AM_PDO_TX_1A00_LEN:], t)
            else:
                t.extended_valid = False
                t.t_ns = t_ns
        self._events(prev, t)
        self.tlm = t
        return copy.copy(t)

    def _events(self, a: Optional[AmTelemetry], b: AmTelemetry):
        tn = b.t_ns
        fault = (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE)
        if a is None or a.state != b.state:
            self.events.append((int(AmEventKind.STATE_CHANGED), int(b.state), tn))
        if a is not None and a.sto_active != b.sto_active:
            self.events.append((int(AmEventKind.STO_CHANGED), 1 if b.sto_active else 0, tn))
        was = a is not None and a.state in fault
        if b.state in fault and not was:
            self.events.append((int(AmEventKind.FAULT), b.error_code, tn))
        if was and b.state not in fault:
            self.events.append((int(AmEventKind.FAULT_CLEARED), 0, tn))
        new_w = b.warning_word & ~(a.warning_word if a is not None else 0)
        if new_w:
            self.events.append((int(AmEventKind.WARNING), new_w, tn))

    def pop_events(self):
        ev, self.events = self.events, []
        return ev

    def comms_lost(self) -> None:
        self.events.append((int(AmEventKind.LINK_LOST), 0x3000, self.tlm.t_ns if self.tlm else 0))

    def authority_ok(self, bus: AmBus) -> bool:
        if self.tlm is not None:
            return bool(self.tlm.statusword & C.SW_REMOTE)
        return self.get_control_bus()[1] == int(bus)

    # ---------------------------------------------------------------- faults
    def fault_info(self) -> AmFaultInfo:
        rd = lambda i, s, f="<I": _u(self._rd(i, s), f)
        fw = rd(0x2600, 1)
        err = rd(0x603F, 0, "<H")
        codes = [0x1000 + n for n in range(32) if fw & (1 << n)]
        if err and err not in codes:
            codes.append(err)
        st = self.tlm.state if self.tlm is not None else AmDriveState.NOT_READY_TO_SWITCH_ON
        return AmFaultInfo(error_code=err, fault_word=fw, first_fault=rd(0x2600, 2), warning_word=rd(0x2602, 1),
                           boot_block=rd(0x2603, 1), safety_io=rd(0x2620, 1, "<B"),
                           latched=bool(fw) or bool(err) or st in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE),
                           codes=codes)

    def fault_history(self) -> List[AmFaultRecord]:
        n = _u(self._rd(0x2601, 0), "<B")
        out = []
        for i in range(1, min(n, 32) + 1):
            code, st, t_ns, vb, iq, tw, tps = struct.unpack("<HBxQiihh", self._rd(0x2601, i)[:24].ljust(24, b"\0"))
            out.append(AmFaultRecord(code=code, drive_state=st, t_ns=t_ns, vbus_V=vb * 1e-3, iq_A=iq * 1e-3,
                                     t_winding_C=tw * 0.1, t_power_stage_C=tps * 0.1))
        return out

    # ---------------------------------------------------------------- limits and gains
    def get_limits(self) -> Tuple[AmLimits, AmLimits]:
        c = self.codec
        rd = lambda i, s, f: _u(self._rd(i, s), f)

        def pos(v):
            return -math.inf if v <= -2 ** 31 else math.inf if v >= 2 ** 31 - 1 else v * 1e-6

        eff = AmLimits(torque_pos_Nm=rd(0x60E0, 0, "<H") * c.rated_torque_Nm * 1e-3,
                       torque_neg_Nm=rd(0x60E1, 0, "<H") * c.rated_torque_Nm * 1e-3,
                       current_A=rd(0x6073, 0, "<H") * c.rated_current_A * 1e-3,
                       speed_rad_s=float(rd(0x2300, 4, "<f")),
                       pos_min_rad=pos(rd(0x607D, 1, "<i")), pos_max_rad=pos(rd(0x607D, 2, "<i")),
                       kp_max_Nm_per_rad=rd(0x2A00, 4, "<I") * 1e-3, kd_max_Nm_s_per_rad=rd(0x2A00, 5, "<I") * 1e-3,
                       comms_loss_ms=rd(0x2300, 20, "<I"))
        peak = self.transp.peak_torque_2s_Nm
        ceil = AmLimits(torque_pos_Nm=peak, torque_neg_Nm=peak, current_A=float(rd(0x2300, 1, "<f")),
                        speed_rad_s=self.transp.no_load_speed_48V_rad_s, pos_min_rad=-math.inf, pos_max_rad=math.inf,
                        kp_max_Nm_per_rad=rd(0x2A50, 1, "<I") * 1e-3, kd_max_Nm_s_per_rad=rd(0x2A50, 2, "<I") * 1e-3,
                        comms_loss_ms=1000)
        return eff, ceil

    def set_limits(self, lim: AmLimits) -> None:
        eff, c = self.get_limits()
        cd = self.codec
        # (value, ceiling, tolerance): the tolerance is one LSB of the wire unit, so the effective
        # limits read back from the drive can always be written again. The drive has the last word.
        checks = [(lim.torque_pos_Nm, c.torque_pos_Nm, cd.rated_torque_Nm * 1e-3),
                  (lim.torque_neg_Nm, c.torque_neg_Nm, cd.rated_torque_Nm * 1e-3),
                  (lim.current_A, c.current_A, cd.rated_current_A * 1e-3), (lim.speed_rad_s, c.speed_rad_s, 1e-3),
                  (lim.kp_max_Nm_per_rad, c.kp_max_Nm_per_rad, 1e-3), (lim.kd_max_Nm_s_per_rad, c.kd_max_Nm_s_per_rad, 1e-3),
                  (lim.comms_loss_ms, c.comms_loss_ms, 0)]
        for v, cap, tol in checks:
            if v is None or (isinstance(v, float) and math.isnan(v)) or v < 0 or v > cap + tol:
                raise AmError(AmStatus.AM_ERR_LIMIT, "value %g above ceiling %g" % (v, cap))
        if lim.pos_min_rad > lim.pos_max_rad:
            raise AmError(AmStatus.AM_ERR_PARAM, "pos_min above pos_max")

        def pos(v):
            if v == -math.inf:
                return -2 ** 31
            if v == math.inf:
                return 2 ** 31 - 1
            return C._q(v, 1e6, *C.I32, "position limit")

        pm_t = lambda v: struct.pack("<H", C._q(v, 1000.0 / cd.rated_torque_Nm, *C.U16, "torque limit"))
        writes = [(0x60E0, 0, pm_t(lim.torque_pos_Nm)), (0x60E1, 0, pm_t(lim.torque_neg_Nm)),
                  (0x6073, 0, struct.pack("<H", C._q(lim.current_A, 1000.0 / cd.rated_current_A, *C.U16, "current limit"))),
                  (0x2300, 4, struct.pack("<f", lim.speed_rad_s)),
                  (0x2A00, 4, struct.pack("<I", C._q(lim.kp_max_Nm_per_rad, 1e3, *C.U32, "K cap"))),
                  (0x2A00, 5, struct.pack("<I", C._q(lim.kd_max_Nm_s_per_rad, 1e3, *C.U32, "D cap"))),
                  (0x2300, 20, struct.pack("<I", int(lim.comms_loss_ms)))]
        pmin, pmax = (0x607D, 1, struct.pack("<i", pos(lim.pos_min_rad))), (0x607D, 2, struct.pack("<i", pos(lim.pos_max_rad)))
        writes += [pmax, pmin] if lim.pos_min_rad > eff.pos_max_rad else [pmin, pmax]
        for i, s, d in writes:
            self._wr(i, s, d, limit_ctx=True)

    def get_loop_gains(self) -> AmLoopGains:
        rf = lambda i, s: float(_u(self._rd(i, s), "<f"))
        return AmLoopGains(pos_kp_1_per_s=rf(0x2A30, 1), vel_kp_Nm_s_per_rad=rf(0x2A30, 2), vel_ki_Nm_per_rad=rf(0x2A30, 3),
                           vel_filter_hz=rf(0x2A40, 1), torque_cmd_filter_hz=rf(0x2A40, 2), notch_hz=rf(0x2A40, 3),
                           notch_q=rf(0x2A40, 4))

    def set_loop_gains(self, g: AmLoopGains) -> None:
        rng = self._rd(0x2A50, 3).ljust(8, b"\0")
        pk_lo, pk_hi = struct.unpack("<ff", rng[:8])
        rf = lambda s: float(_u(self._rd(0x2A50, s), "<f"))
        vp, vi, fmin = rf(4), rf(5), rf(6)
        vals = (g.pos_kp_1_per_s, g.vel_kp_Nm_s_per_rad, g.vel_ki_Nm_per_rad, g.vel_filter_hz, g.torque_cmd_filter_hz,
                g.notch_hz, g.notch_q)
        if any(v is None or not math.isfinite(v) for v in vals):
            raise AmError(AmStatus.AM_ERR_PARAM, "non-finite gain")
        if not (pk_lo <= g.pos_kp_1_per_s <= pk_hi) or not (0 <= g.vel_kp_Nm_s_per_rad <= vp) \
                or not (0 <= g.vel_ki_Nm_per_rad <= vi) or g.vel_filter_hz < fmin \
                or (g.torque_cmd_filter_hz != 0.0 and g.torque_cmd_filter_hz < fmin):
            raise AmError(AmStatus.AM_ERR_LIMIT, "gain or filter outside the envelope")
        for (i, s), v in zip(((0x2A30, 1), (0x2A30, 2), (0x2A30, 3), (0x2A40, 1), (0x2A40, 2), (0x2A40, 3), (0x2A40, 4)), vals):
            self._wr(i, s, struct.pack("<f", v), limit_ctx=True)

    # ---------------------------------------------------------------- configuration
    def sdo_read(self, index: int, sub: int) -> bytes:
        return self._rd(index, sub)

    def sdo_write(self, index: int, sub: int, data: bytes) -> None:
        self._wr(index, sub, data)

    def store_parameters(self) -> None:
        self._wr(0x1010, 1, struct.pack("<I", STORE_SIGNATURE), limit_ctx=True)

    def restore_defaults(self) -> None:
        self._wr(0x1011, 1, struct.pack("<I", RESTORE_SIGNATURE), limit_ctx=True)
        self.ident.user_label = self._rd(0x2E00, 1).decode(errors="replace").rstrip("\0")

    def set_user_label(self, label: str) -> None:
        data = label.encode()
        if len(data) > 31:
            raise AmError(AmStatus.AM_ERR_PARAM, "label longer than 31 characters")
        self._wr(0x2E00, 1, data, limit_ctx=True)
        self.ident.user_label = label

    def get_control_bus(self) -> Tuple[int, int]:
        return _u(self._rd(0x2901, 1), "<B"), _u(self._rd(0x2901, 3), "<B")

    def set_control_bus(self, bus: int) -> None:
        if bus not in (int(AmBus.NONE), int(AmBus.ETHERCAT), int(AmBus.CANFD)):
            raise AmError(AmStatus.AM_ERR_PARAM, "bus must be NONE, ETHERCAT or CANFD")
        self._wr(0x2901, 1, bytes([bus]), limit_ctx=True)

    def read_calibration(self) -> str:
        return bytes(self.tp.file_read(self.address, "am_cal.json")).decode(errors="replace")

    def last_abort_code(self) -> int:
        return self.last_abort


class TransportBackend(Backend):
    """Backend over any Transport (EtherCAT, CAN-FD, the wire-level simulator or your own)."""

    def __init__(self, transport: Transport):
        self.transport = transport
        self.name = transport.name
        self.joints: List[TransportJoint] = []
        self.cfg: Optional[AmSessionConfig] = None
        self.profile = "pdo"
        self._next = None

    def open(self, cfg: AmSessionConfig) -> None:
        self.cfg = cfg
        self.profile = "compact" if (cfg.bus == AmBus.CANFD and cfg.canfd_compact) else "pdo"
        self.transport.open(cfg)

    def close(self) -> None:
        self.transport.close()

    def _addresses(self) -> List[int]:
        found = self.transport.probe()
        if found is not None:
            return list(found)
        rng = CAN_NODE_IDS if self.cfg.bus == AmBus.CANFD else range(0, 256)
        out = []
        for a in rng:
            try:
                self.transport.sdo_read(a, 0x1018, 1)
                out.append(a)
            except (SdoAbort, AmError):
                if self.cfg.bus != AmBus.CANFD:
                    break
        return out

    def scan(self) -> List[AmDeviceInfo]:
        out = []
        for a in self._addresses():
            def rd(i, s, f="<I"):
                return _u(self.transport.sdo_read(a, i, s), f)
            try:
                vid, pc, rev, ser = rd(0x1018, 1), rd(0x1018, 2), rd(0x1018, 3), rd(0x1018, 4)
            except (SdoAbort, AmError):
                continue
            try:
                odv = rd(0x2000, 6)
                eff = rd(0x2901, 3, "<B")
                is_am = True
            except (SdoAbort, AmError):
                odv, eff, is_am = 0, 0, False
            out.append(AmDeviceInfo(bus=self.cfg.bus, address=a, vendor_id=vid, product_code=pc, revision=rev,
                                    serial=ser, is_am_unit=is_am, od_compatible=is_am and _od_ok(odv),
                                    has_control_authority=is_am and eff == int(self.cfg.bus)))
        return out

    def open_joint(self, address: int) -> TransportJoint:
        try:
            self.transport.sdo_read(address, 0x1018, 1)
        except (SdoAbort, AmError):
            raise AmError(AmStatus.AM_ERR_NO_DEVICE, "no unit at address %d" % address)
        j = TransportJoint(self, address)
        self.joints.append(j)
        return j

    def start(self) -> None:
        self.transport.setup([j.address for j in self.joints], self.profile, self.cfg.telemetry_extended)
        self.transport.start()
        self._next = None

    def stop(self) -> None:
        self.transport.stop()
        for j in self.joints:
            j.comms_lost()

    def cycle(self, items: Sequence[CycleItem]) -> List[Optional[AmTelemetry]]:
        if self.transport.realtime:
            period = self.cfg.cycle_us * 1e-6
            now = time.perf_counter()
            if self._next is None or now - self._next > 10 * period:
                self._next = now
            elif self._next > now:
                time.sleep(self._next - now)
            self._next += period
        outs = [jb.pack(cw, mode, cmd, fresh) for jb, cw, mode, cmd, fresh in items]
        ins, t_ns = self.transport.pdo_exchange(outs)
        return [jb.unpack(b, t_ns) for (jb, _, _, _, _), b in zip(items, ins)]


def _od_ok(od_version: int) -> bool:
    major = (od_version >> 24) & 0xFF
    return major == OD_MAJOR_SUPPORTED or (OD_MAJOR_SUPPORTED > 0 and major == OD_MAJOR_SUPPORTED - 1)
