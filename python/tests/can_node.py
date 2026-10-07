"""Simulated CAN-FD joints on the python-can virtual bus, for the CAN-FD transport tests.

A thread answers NMT, the SDO protocol (CiA 301 expedited and segmented), group command frames
(compact profile) and RxPDO + SYNC (profile 0), with am_sdk SimNode drives behind it.
"""
import struct
import threading

import can

from am_sdk.am_types import AmBus, AmSessionConfig
from am_sdk.backends.simwire import SimNode
from am_sdk.backends.transport import SdoAbort


class CanSimBus:
    def __init__(self, units, channel="am_test", cycle_us=1000, options=None):
        cfg = AmSessionConfig(bus=AmBus.CANFD, cycle_us=cycle_us)
        options = options or [None] * len(units)
        self.nodes = {i + 1: SimNode(u, i + 1, cfg, options[i]) for i, u in enumerate(units)}
        self.profile = {n: 0 for n in self.nodes}
        self.operational = {n: False for n in self.nodes}
        self.rpdo = {}
        self.seg = {}
        self.bus = can.Bus(interface="virtual", channel=channel, fd=True)
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop_evt.set()
        self.thread.join(1.0)
        self.bus.shutdown()

    def _send(self, cob, data, fd=True):
        if fd:
            for size in (8, 12, 16, 20, 24, 32, 48, 64):
                if len(data) <= size:
                    data = data.ljust(size, b"\0")
                    break
        self.bus.send(can.Message(arbitration_id=cob, data=data, is_extended_id=False, is_fd=fd, bitrate_switch=fd))

    def _run(self):
        while not self.stop_evt.is_set():
            m = self.bus.recv(0.05)
            if m is None:
                continue
            cob, data = m.arbitration_id, bytes(m.data)
            if cob == 0x000 and len(data) >= 2:
                targets = list(self.nodes) if data[1] == 0 else [data[1]]
                for n in targets:
                    if n in self.nodes:
                        self.operational[n] = data[0] == 0x01
            elif 0x601 <= cob <= 0x63F and (cob - 0x600) in self.nodes:
                self._sdo(cob - 0x600, data)
            elif 0x201 <= cob <= 0x23F and (cob - 0x200) in self.nodes:
                self.rpdo[cob - 0x200] = data[:28]
            elif cob == 0x080:
                for n, rx in sorted(self.rpdo.items()):
                    if self.operational[n] and self.profile[n] == 0:
                        out = self.nodes[n].pdo(rx, True)
                        self._send(0x180 + n, out[:24])
                        self._send(0x280 + n, out[24:])
                self.rpdo = {}
            elif 0x1C0 <= cob <= 0x1C3:
                g = cob - 0x1C0
                for n, node in sorted(self.nodes.items()):
                    if n // 4 == g and self.operational[n] and self.profile[n] in (1, 2):
                        slot = n % 4
                        fb = node.compact(data[16 * slot:16 * slot + 16], self.profile[n] == 2)
                        self._send(0x180 + n, fb)

    def _abort(self, n, mux, code):
        self._send(0x580 + n, b"\x80" + mux + struct.pack("<I", code), fd=False)

    def _sdo(self, n, d):
        node = self.nodes[n]
        cs = d[0]
        mux = d[1:4]
        idx, sub = struct.unpack("<HB", mux)
        try:
            if cs == 0x40:                                   # upload initiate
                if (idx, sub) == (0x2902, 1):
                    data = bytes([self.profile[n]])
                else:
                    data = node.sdo_read(idx, sub)
                if len(data) <= 4:
                    self._send(0x580 + n, bytes([0x43 | ((4 - len(data)) << 2)]) + mux + data.ljust(4, b"\0"), fd=False)
                else:
                    self.seg[n] = ["up", data, 0, mux]
                    self._send(0x580 + n, b"\x41" + mux + struct.pack("<I", len(data)), fd=False)
            elif cs & 0xEF == 0x60 and self.seg.get(n, [None])[0] == "up":      # upload segment
                _, data, pos, m = self.seg[n]
                part = data[pos:pos + 7]
                pos += len(part)
                last = pos >= len(data)
                t = cs & 0x10
                self._send(0x580 + n, bytes([t | ((7 - len(part)) << 1) | (1 if last else 0)]) + part.ljust(7, b"\0"), fd=False)
                if last:
                    del self.seg[n]
                else:
                    self.seg[n][2] = pos
            elif cs & 0xE3 == 0x23:                          # expedited download
                size = 4 - ((cs >> 2) & 3) if cs & 1 else 4
                self._write(n, idx, sub, d[4:4 + size])
                self._send(0x580 + n, b"\x60" + mux + b"\0\0\0\0", fd=False)
            elif cs == 0x21:                                 # segmented download initiate
                self.seg[n] = ["down", bytearray(), struct.unpack("<I", d[4:8])[0], mux]
                self._send(0x580 + n, b"\x60" + mux + b"\0\0\0\0", fd=False)
            elif cs & 0xE0 == 0x00 and self.seg.get(n, [None])[0] == "down":    # download segment
                _, buf, size, m = self.seg[n]
                k = 7 - ((cs >> 1) & 7)
                buf += d[1:1 + k]
                if cs & 1:
                    del self.seg[n]
                    i2, s2 = struct.unpack("<HB", m)
                    self._write(n, i2, s2, bytes(buf[:size]))
                self._send(0x580 + n, bytes([0x20 | (cs & 0x10)]) + b"\0" * 7, fd=False)
            else:
                self._abort(n, mux, 0x05040001)
        except SdoAbort as e:
            self.seg.pop(n, None)
            self._abort(n, mux, e.code)

    def _write(self, n, idx, sub, data):
        if (idx, sub) == (0x2902, 1):
            self.profile[n] = data[0]
            return
        self.nodes[n].sdo_write(idx, sub, data)
