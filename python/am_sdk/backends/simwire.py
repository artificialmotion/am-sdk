# SPDX-License-Identifier: Apache-2.0
"""Wire-level simulator: the sim joints behind real PDO and object-dictionary bytes.

SimWireTransport is a Transport, so a session on it runs the full bus path (codec, process images,
object reads and writes) without hardware. Use it to test your own integration code, or to see
what a joint sends on the wire:

    import am_sdk as am
    from am_sdk.backends import SimWireTransport, TransportBackend
    tp = SimWireTransport(["M"])
    s = am.am_init(am.AmSessionConfig(backend=TransportBackend(tp)))

SimNode wraps one SimJoint and answers byte-level requests; the EtherCAT and CAN-FD test
fixtures use it as the device on the other side of the bus.
"""
from __future__ import annotations

from typing import List, Optional, Tuple, Union

from .. import codec as C
from ..am_types import AmCommand, AmDriveState, AmError, AmMode, AmSessionConfig, AmStatus
from .sim import SimJoint, SimOptions
from .transport import SdoAbort, Transport


class SimNode:
    """One simulated drive at the byte level."""

    def __init__(self, unit: str, address: int, cfg: AmSessionConfig, opts: Optional[SimOptions] = None):
        self.joint = SimJoint(unit, address, cfg, opts)
        self.address = address
        self.bus = cfg.bus
        rated = self.joint.ud.rated_nm
        self.codec = C.am_codec_init(int(round(rated * 1e3)), int(round(self.joint.rated_current_A * 1e3)), 20, 1,
                                     address % 4)
        self._enable_level = False
        self._mode = AmMode.NONE

    # ---------------------------------------------------------------- objects
    def sdo_read(self, index: int, sub: int) -> bytes:
        try:
            return self.joint.sdo_read(index, sub)
        except AmError:
            raise SdoAbort(self.joint.last_abort or 0x08000000)

    def sdo_write(self, index: int, sub: int, data: bytes) -> None:
        try:
            self.joint.sdo_write(index, sub, data)
        except AmError as e:
            code = self.joint.last_abort
            if e.status == AmStatus.AM_ERR_LIMIT:
                code = 0x06090031
            elif e.status == AmStatus.AM_ERR_PARAM:
                code = 0x06090030
            elif e.status == AmStatus.AM_ERR_STATE:
                code = 0x08000022
            raise SdoAbort(code or 0x08000000)

    # ---------------------------------------------------------------- cyclic
    def pdo(self, rx: bytes, extended: bool) -> bytes:
        """RxPDO 0x1601 in, TxPDO 0x1A00 (+ 0x1A01) out."""
        cw, cmd = C.drive_unpack_rx1601(rx)
        t = self.joint.exchange(cw, cmd.mode, cmd, True)
        out = C.drive_pack_tx1a00(self.codec, t)
        if extended:
            out += C.drive_pack_tx1a01(t)
        return out

    def compact(self, slot: bytes, extended: bool) -> bytes:
        """One group-frame slot in, compact or estimator-lite feedback out. The drive turns the control
        byte into the same controlword walk as on EtherCAT."""
        control, counter, cmd = C.drive_unpack_cfd_slot(self.codec, slot)
        if cmd.mode == AmMode.NONE:
            cmd.mode = self._mode
        self._mode = cmd.mode
        j = self.joint
        sw = j._statusword()
        st = j.state
        if control & C.AM_CC_FAULT_RESET:
            cw = C.CW_FAULT_RESET
        elif control & C.AM_CC_QUICK_STOP:
            cw = C.CW_QUICK_STOP
        elif control & C.AM_CC_ENABLE:
            cw = C.am_codec_next_controlword(sw, True)
        elif st in (AmDriveState.READY_TO_SWITCH_ON, AmDriveState.SWITCHED_ON, AmDriveState.OPERATION_ENABLED):
            cw = C.CW_SHUTDOWN
        else:
            cw = C.CW_DISABLE_VOLTAGE
        if control & C.AM_CC_ZERO_TORQUE:
            cmd = AmCommand(mode=cmd.mode)
        t = j.exchange(cw, cmd.mode, cmd, True)
        return C.drive_pack_cfd_fb(t, counter, j._authority == j.cfg_bus, extended)


class SimWireTransport(Transport):
    """In-process simulated bus. Addresses are 0..n-1 (EtherCAT ring positions) as in the sim backend."""

    name = "sim-wire"
    realtime = False

    def __init__(self, units: Optional[List[str]] = None, options: Optional[List[Union[SimOptions, dict]]] = None):
        self._units = units
        self._options = options
        self.nodes: List[SimNode] = []
        self.order: List[int] = []
        self.profile = "pdo"
        self.extended = True
        self.cfg: Optional[AmSessionConfig] = None

    def open(self, cfg: AmSessionConfig) -> None:
        self.cfg = cfg
        units = self._units or cfg.sim_units or ["M"]
        opts = self._options or cfg.sim_options or []
        self.nodes = []
        for i, u in enumerate(units):
            o = opts[i] if i < len(opts) else SimOptions()
            if isinstance(o, dict):
                o = SimOptions(**o)
            self.nodes.append(SimNode(u, i, cfg, o))

    def node(self, address: int) -> SimNode:
        if not 0 <= address < len(self.nodes):
            raise SdoAbort(0x05040000, "no node")       # SDO protocol timed out: nobody answers
        return self.nodes[address]

    def probe(self) -> Optional[List[int]]:
        return list(range(len(self.nodes)))

    def setup(self, addresses: List[int], profile: str, extended: bool) -> None:
        self.order, self.profile, self.extended = list(addresses), profile, extended

    def stop(self) -> None:
        for a in self.order:
            self.nodes[a].joint.comms_lost()

    def pdo_exchange(self, out: List[bytes]) -> Tuple[List[Optional[bytes]], int]:
        ins: List[Optional[bytes]] = []
        for a, o in zip(self.order, out):
            n = self.nodes[a]
            ins.append(n.compact(o, self.extended) if self.profile == "compact" else n.pdo(o, self.extended))
        t_ns = self.nodes[self.order[0]].joint.t_ns if self.order else 0
        return ins, t_ns

    def sdo_read(self, address: int, index: int, sub: int) -> bytes:
        return self.node(address).sdo_read(index, sub)

    def sdo_write(self, address: int, index: int, sub: int, data: bytes) -> None:
        self.node(address).sdo_write(index, sub, data)

    def file_read(self, address: int, name: str) -> bytes:
        if name != "am_cal.json":
            raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "no file %r" % name)
        return self.node(address).joint.read_calibration().encode()
