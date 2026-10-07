# SPDX-License-Identifier: Apache-2.0
"""CAN-FD transport on python-can. Install with: pip install "am-sdk[can]".

    cfg = am.AmSessionConfig(bus=am.AmBus.CANFD, interface="can:socketcan:can0", canfd_compact=True)
    cfg = am.AmSessionConfig(bus=am.AmBus.CANFD, interface="can:pcan:PCAN_USBBUS1@1000000/5000000")

Interface string: can:<python-can interface>:<channel>[@<nominal bit/s>/<data bit/s>], default 1/5 Mbit/s.
Node ids are 1..63.

Cyclic traffic (docs/API.md section 3):
    compact profile (canfd_compact=True): one 64 B group command frame per group of up to 4 joints on
        0x1C0 + group; each joint answers with 16 B (or 32 B with telemetry_extended) on 0x180 + node.
    profile 0: RxPDO 0x1601 on 0x200 + node (28 B in a 32 B frame), then SYNC; TxPDO 0x1A00 on
        0x180 + node and, with telemetry_extended, TxPDO 0x1A01 on 0x280 + node (60 B in a 64 B frame).

Object access uses the CiA 301 SDO protocol (expedited and segmented transfer) on 0x600 + node
(request) and 0x580 + node (response), in classic 8-byte frames.
"""
from __future__ import annotations

import struct
import time
from typing import Dict, List, Optional, Tuple

from ..am_types import AmError, AmSessionConfig, AmStatus
from .transport import SdoAbort, Transport

ID_NMT = 0x000
ID_SYNC = 0x080
ID_TPDO1 = 0x180
ID_RPDO1 = 0x200
ID_TPDO2 = 0x280
ID_SDO_TX = 0x580          # device -> host
ID_SDO_RX = 0x600          # host -> device
NMT_START = 0x01
NMT_PREOP = 0x80


def _can():
    try:
        import can
    except ImportError as e:  # pragma: no cover
        raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, 'python-can is not installed: pip install "am-sdk[can]"') from e
    return can


def parse_can_interface(spec: str) -> Tuple[str, str, int, int]:
    """"can:pcan:PCAN_USBBUS1@1000000/5000000" -> ("pcan", "PCAN_USBBUS1", 1000000, 5000000)."""
    body = spec[4:] if spec.startswith("can:") else spec
    body, _, rates = body.partition("@")
    iface, _, channel = body.partition(":")
    nom, _, dat = rates.partition("/")
    return iface, channel, int(nom or 1000000), int(dat or 5000000)


def _fd_len(n: int) -> int:
    for size in (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64):
        if n <= size:
            return size
    raise ValueError("CAN-FD payload above 64 bytes")


class CanFdTransport(Transport):
    name = "canfd"
    realtime = True

    def __init__(self, interface: str = "virtual", channel: str = "am", nominal: int = 1000000, data: int = 5000000,
                 bus=None, rx_timeout_s: Optional[float] = None):
        self.interface, self.channel, self.nominal, self.data = interface, channel, nominal, data
        self.bus = bus
        self._own_bus = bus is None
        self.rx_timeout_s = rx_timeout_s
        self.order: List[int] = []
        self.profile = "pdo"
        self.extended = True
        self.groups: Dict[int, Dict[int, int]] = {}     # cob id -> {node: slot}
        self.sdo_timeout = 0.1

    # ---------------------------------------------------------------- setup
    def open(self, cfg: AmSessionConfig) -> None:
        can = _can()
        self._can = can
        if cfg.canfd_nominal_bps:
            self.nominal = cfg.canfd_nominal_bps
        if cfg.canfd_data_bps:
            self.data = cfg.canfd_data_bps
        if self.bus is None:
            kw = dict(interface=self.interface, channel=self.channel, fd=True)
            if self.interface not in ("virtual", "socketcan"):
                kw.update(bitrate=self.nominal, data_bitrate=self.data)
            self.bus = can.Bus(**kw)
        self.sdo_timeout = (cfg.sdo_timeout_ms or 100) * 1e-3
        if self.rx_timeout_s is None:
            self.rx_timeout_s = max(cfg.cycle_us * 1e-6 * 0.9, 0.0005)

    def close(self) -> None:
        if self._own_bus and self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def _send(self, cob: int, data: bytes, fd: bool = True) -> None:
        if fd:
            data = bytes(data).ljust(_fd_len(len(data)), b"\0")
        msg = self._can.Message(arbitration_id=cob, data=data, is_extended_id=False, is_fd=fd, bitrate_switch=fd)
        try:
            self.bus.send(msg, timeout=0.05)
        except Exception as e:
            raise AmError(AmStatus.AM_ERR_BUS, "CAN send 0x%03X failed: %s" % (cob, e)) from e

    def _nmt(self, cmd: int, node: int) -> None:
        self._send(ID_NMT, bytes([cmd, node]), fd=False)

    def setup(self, addresses: List[int], profile: str, extended: bool) -> None:
        self.order, self.profile, self.extended = list(addresses), profile, extended
        self.groups = {}
        for n in self.order:
            sel = 0 if profile != "compact" else (2 if extended else 1)
            self.sdo_write(n, 0x2902, 1, bytes([sel]))
            if profile == "compact":
                cob = struct.unpack("<I", self.sdo_read(n, 0x2902, 2)[:4].ljust(4, b"\0"))[0] & 0x7FF
                slot = self.sdo_read(n, 0x2902, 3)[0] & 0x3
                self.groups.setdefault(cob, {})[n] = slot

    def start(self) -> None:
        for n in self.order:
            self._nmt(NMT_START, n)

    def stop(self) -> None:
        for n in self.order:
            try:
                self._nmt(NMT_PREOP, n)
            except AmError:
                pass

    # ---------------------------------------------------------------- cyclic
    def pdo_exchange(self, out: List[bytes]) -> Tuple[List[Optional[bytes]], int]:
        idx = {n: i for i, n in enumerate(self.order)}
        if self.profile == "compact":
            for cob, members in sorted(self.groups.items()):
                frame = bytearray(64)
                for n, slot in members.items():
                    frame[16 * slot:16 * slot + 16] = out[idx[n]][:16]
                self._send(cob, bytes(frame))
            want = {ID_TPDO1 + n: n for n in self.order}
            flen = 32 if self.extended else 16
        else:
            for n in self.order:
                self._send(ID_RPDO1 + n, out[idx[n]])
            self._send(ID_SYNC, b"", fd=False)
            want = {ID_TPDO1 + n: n for n in self.order}
            if self.extended:
                want.update({ID_TPDO2 + n: n for n in self.order})
            flen = 24
        t_ns = time.monotonic_ns()
        got: Dict[int, bytes] = {}
        end = time.monotonic() + self.rx_timeout_s
        while len(got) < len(want):
            m = self.bus.recv(max(end - time.monotonic(), 0.0))
            if m is None:
                break
            if m.arbitration_id in want and m.arbitration_id not in got:
                got[m.arbitration_id] = bytes(m.data)
        ins: List[Optional[bytes]] = []
        for n in self.order:
            a = got.get(ID_TPDO1 + n)
            if a is None:
                ins.append(None)
            elif self.profile == "compact":
                ins.append(a[:flen])
            else:
                b = got.get(ID_TPDO2 + n) if self.extended else None
                ins.append(a[:24] + (b[:60] if b is not None else b""))
        return ins, t_ns

    # ---------------------------------------------------------------- SDO client (CiA 301 framing)
    def _sdo_req(self, node: int, data: bytes) -> bytes:
        self._send(ID_SDO_RX + node, data.ljust(8, b"\0"), fd=False)
        end = time.monotonic() + self.sdo_timeout
        while True:
            m = self.bus.recv(max(end - time.monotonic(), 0.0))
            if m is None:
                raise SdoAbort(0x05040000, "SDO timeout on node %d" % node)
            if m.arbitration_id == ID_SDO_TX + node and len(m.data) >= 8:
                r = bytes(m.data[:8])
                if r[0] == 0x80:
                    raise SdoAbort(struct.unpack("<I", r[4:8])[0])
                return r

    def sdo_read(self, address: int, index: int, sub: int) -> bytes:
        mux = struct.pack("<HB", index, sub)
        r = self._sdo_req(address, b"\x40" + mux)
        cs = r[0]
        if cs & 0xE0 != 0x40:
            raise SdoAbort(0x05040001, "unexpected SDO response")
        if cs & 0x02:                                   # expedited
            n = 4 - ((cs >> 2) & 0x3) if cs & 0x01 else 4
            return r[4:4 + n]
        size = struct.unpack("<I", r[4:8])[0] if cs & 0x01 else None
        data = bytearray()
        toggle = 0
        while True:
            seg = self._sdo_req(address, bytes([0x60 | (toggle << 4)]))
            if (seg[0] >> 4) & 1 != toggle or seg[0] & 0xE0:
                raise SdoAbort(0x05030000, "toggle bit not alternated")
            n = 7 - ((seg[0] >> 1) & 0x7)
            data += seg[1:1 + n]
            if seg[0] & 0x01:
                break
            toggle ^= 1
        return bytes(data[:size] if size is not None else data)

    def sdo_write(self, address: int, index: int, sub: int, data: bytes) -> None:
        mux = struct.pack("<HB", index, sub)
        data = bytes(data)
        if 0 < len(data) <= 4:
            cs = 0x23 | ((4 - len(data)) << 2)
            r = self._sdo_req(address, bytes([cs]) + mux + data.ljust(4, b"\0"))
            if r[0] != 0x60:
                raise SdoAbort(0x05040001, "unexpected SDO response")
            return
        r = self._sdo_req(address, b"\x21" + mux + struct.pack("<I", len(data)))
        if r[0] != 0x60:
            raise SdoAbort(0x05040001, "unexpected SDO response")
        toggle, pos = 0, 0
        while True:
            part = data[pos:pos + 7]
            pos += len(part)
            last = pos >= len(data)
            cs = (toggle << 4) | ((7 - len(part)) << 1) | (1 if last else 0)
            r = self._sdo_req(address, bytes([cs]) + part.ljust(7, b"\0"))
            if r[0] != (0x20 | (toggle << 4)):
                raise SdoAbort(0x05030000, "toggle bit not alternated")
            if last:
                return
            toggle ^= 1
