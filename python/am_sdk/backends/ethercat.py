# SPDX-License-Identifier: Apache-2.0
"""EtherCAT transport on pysoem (SOEM). Install with: pip install "am-sdk[ecat]".

    cfg = am.AmSessionConfig(bus=am.AmBus.ETHERCAT, interface="ecat:eth1")          # Linux
    cfg = am.AmSessionConfig(bus=am.AmBus.ETHERCAT, interface=r"ecat:\\Device\\NPF_{...}")   # Windows

The NIC must be dedicated to the EtherCAT line. Linux needs CAP_NET_RAW (run as root or
`setcap cap_net_raw+ep` on the interpreter); Windows needs Npcap. `list_nics()` prints names.
Addresses are 0-based ring positions. Every joint gets RxPDO 0x1601 and TxPDO 0x1A00
(+ 0x1A01 when telemetry_extended is set). Python paces the cycle with the host clock: use it for
commissioning, identification and tests. Run 1 kHz control from C.
"""
from __future__ import annotations

import struct
import time
from typing import List, Optional, Tuple

from ..am_types import AmError, AmSessionConfig, AmStatus
from .transport import SdoAbort, Transport


def _soem(mod=None):
    if mod is not None:
        return mod
    try:
        import pysoem
    except ImportError as e:  # pragma: no cover
        raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED,
                      'pysoem is not installed: pip install "am-sdk[ecat]" (Windows also needs Npcap)') from e
    return pysoem


def list_nics(pysoem_module=None) -> List[Tuple[str, str]]:
    """(name, description) of every network adapter pysoem can open."""
    ps = _soem(pysoem_module)
    dec = lambda v: v.decode("ascii", "replace") if isinstance(v, bytes) else str(v)
    return [(dec(a.name), dec(a.desc)) for a in ps.find_adapters()]


class EtherCatTransport(Transport):
    name = "ethercat"
    realtime = True

    def __init__(self, ifname: str, pysoem_module=None, rx_timeout_us: int = 2000):
        self.ifname = ifname
        self.ps = _soem(pysoem_module)
        self.rx_timeout_us = rx_timeout_us
        self.master = None
        self.order: List[int] = []
        self.extended = True

    def open(self, cfg: AmSessionConfig) -> None:
        self.master = self.ps.Master()
        try:
            self.master.open(self.ifname)
        except Exception as e:
            raise AmError(AmStatus.AM_ERR_BUS, "cannot open NIC %r (%s); Windows needs Npcap, Linux needs "
                                               "CAP_NET_RAW" % (self.ifname, e)) from e
        n = self.master.config_init()
        if n <= 0:
            raise AmError(AmStatus.AM_ERR_NO_DEVICE, "no EtherCAT slaves found on %r" % self.ifname)

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def probe(self) -> Optional[List[int]]:
        return list(range(len(self.master.slaves)))

    def _slave(self, address: int):
        if not 0 <= address < len(self.master.slaves):
            raise SdoAbort(0x05040000, "no slave at ring position %d" % address)
        return self.master.slaves[address]

    def sdo_read(self, address: int, index: int, sub: int) -> bytes:
        sl = self._slave(address)
        try:
            return bytes(sl.sdo_read(index, sub))
        except self.ps.SdoError as e:
            raise SdoAbort(getattr(e, "abort_code", 0) or 0x08000000)
        except Exception as e:
            raise AmError(AmStatus.AM_ERR_BUS, "SDO read 0x%04X:%d: %s" % (index, sub, e)) from e

    def sdo_write(self, address: int, index: int, sub: int, data: bytes) -> None:
        sl = self._slave(address)
        try:
            sl.sdo_write(index, sub, bytes(data))
        except self.ps.SdoError as e:
            raise SdoAbort(getattr(e, "abort_code", 0) or 0x08000000)
        except Exception as e:
            raise AmError(AmStatus.AM_ERR_BUS, "SDO write 0x%04X:%d: %s" % (index, sub, e)) from e

    def file_read(self, address: int, name: str) -> bytes:
        sl = self._slave(address)
        try:
            return bytes(sl.foe_read(name, 0, 65536))
        except Exception as e:
            raise AmError(AmStatus.AM_ERR_NOT_SUPPORTED, "FoE read %r: %s" % (name, e)) from e

    def _state(self, want: int, timeout_s: float, exchange: bool = False) -> None:
        m = self.master
        m.state = want
        m.write_state()
        end = time.monotonic() + timeout_s
        while True:
            if exchange:
                m.send_processdata()
                m.receive_processdata(self.rx_timeout_us)
            if m.state_check(want, 50000) == want:
                return
            if time.monotonic() > end:
                m.read_state()
                bad = ["%d: 0x%02X AL 0x%04X" % (i, s.state, s.al_status) for i, s in enumerate(m.slaves)
                       if s.state != want]
                raise AmError(AmStatus.AM_ERR_BUS, "slaves did not reach state 0x%02X (%s)" % (want, ", ".join(bad)))

    def setup(self, addresses: List[int], profile: str, extended: bool) -> None:
        self.order, self.extended = list(addresses), extended
        for a in self.order:
            # fixed PDO content in v1: choose 0x1601 and 0x1A00 (+ 0x1A01) in PRE-OP
            self.sdo_write(a, 0x1C12, 0, b"\x00")
            self.sdo_write(a, 0x1C12, 1, struct.pack("<H", 0x1601))
            self.sdo_write(a, 0x1C12, 0, b"\x01")
            self.sdo_write(a, 0x1C13, 0, b"\x00")
            self.sdo_write(a, 0x1C13, 1, struct.pack("<H", 0x1A00))
            if extended:
                self.sdo_write(a, 0x1C13, 2, struct.pack("<H", 0x1A01))
            self.sdo_write(a, 0x1C13, 0, bytes([2 if extended else 1]))
        self.master.config_map()
        if hasattr(self.master, "config_dc"):
            try:
                self.master.config_dc()
            except Exception:
                pass
        self._state(self.ps.SAFEOP_STATE, 3.0)

    def start(self) -> None:
        self._state(self.ps.OP_STATE, 3.0, exchange=True)

    def stop(self) -> None:
        try:
            self._state(self.ps.SAFEOP_STATE, 1.0)
        except AmError:
            pass

    def pdo_exchange(self, out: List[bytes]) -> Tuple[List[Optional[bytes]], int]:
        m = self.master
        for a, o in zip(self.order, out):
            m.slaves[a].output = bytes(o)
        m.send_processdata()
        wkc = m.receive_processdata(self.rx_timeout_us)
        t_ns = int(getattr(m, "dc_time", 0) or time.monotonic_ns())
        if wkc < m.expected_wkc:
            return [None] * len(self.order), t_ns
        return [bytes(m.slaves[a].input) for a in self.order], t_ns
