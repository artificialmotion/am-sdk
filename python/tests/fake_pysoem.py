# SPDX-License-Identifier: Apache-2.0
"""A stand-in for the pysoem module, backed by am_sdk SimNode drives, so the EtherCAT transport runs without a NIC.

It mirrors the pysoem calls EtherCatTransport uses (checked against pysoem 1.1.13): Master.open, config_init,
config_map, config_dc, state / write_state / state_check / read_state, send_processdata, receive_processdata,
expected_wkc, slaves[i].sdo_read / sdo_write / output / input / state / al_status, SdoError.abort_code.
"""
import struct

INIT_STATE, PREOP_STATE, BOOT_STATE, SAFEOP_STATE, OP_STATE = 1, 2, 3, 4, 8


class SdoError(Exception):
    def __init__(self, slave_pos, index, subindex, abort_code, desc=""):
        super().__init__("SDO abort 0x%08X" % abort_code)
        self.slave_pos, self.index, self.subindex, self.abort_code, self.desc = slave_pos, index, subindex, abort_code, desc


class FakeSlave:
    def __init__(self, pos, node, extended_default=True):
        self.pos, self.node = pos, node
        self.state = PREOP_STATE
        self.al_status = 0
        self.output = b""
        self.input = b""
        self.rx = [0x1601]
        self.tx = [0x1A00, 0x1A01] if extended_default else [0x1A00]
        self._n = {0x1C12: 1, 0x1C13: len(self.tx)}

    def sdo_read(self, index, subindex, size=0, ca=False):
        from am_sdk.backends.transport import SdoAbort
        try:
            return self.node.sdo_read(index, subindex)
        except SdoAbort as e:
            raise SdoError(self.pos, index, subindex, e.code)

    def sdo_write(self, index, subindex, data, ca=False):
        from am_sdk.backends.transport import SdoAbort
        if index in (0x1C12, 0x1C13):
            if self.state != PREOP_STATE:
                raise SdoError(self.pos, index, subindex, 0x08000022)
            lst = self.rx if index == 0x1C12 else self.tx
            if subindex == 0:
                n = data[0]
                del lst[n:]
                self._n[index] = n
            else:
                v = struct.unpack("<H", bytes(data))[0]
                while len(lst) < subindex:
                    lst.append(0)
                lst[subindex - 1] = v
            return
        try:
            self.node.sdo_write(index, subindex, bytes(data))
        except SdoAbort as e:
            raise SdoError(self.pos, index, subindex, e.code)

    def state_check(self, expected, timeout=2000):
        return self.state

    def write_state(self):
        pass


class Adapter:
    def __init__(self, name, desc):
        self.name, self.desc = name, desc


class Master:
    nodes = []          # set by the test: list of SimNode

    def __init__(self):
        self.slaves = []
        self.state = INIT_STATE
        self.expected_wkc = 0
        self.dc_time = 0
        self.mapped = False

    def open(self, ifname):
        self.ifname = ifname

    def config_init(self):
        self.slaves = [FakeSlave(i, n) for i, n in enumerate(Master.nodes)]
        return len(self.slaves)

    def config_map(self):
        if not all(s.rx == [0x1601] for s in self.slaves):
            raise RuntimeError("fake supports RxPDO 0x1601 only")
        self.mapped = True
        for s in self.slaves:
            s.output = b"\0" * 28
        self.expected_wkc = 3 * len(self.slaves)
        return 0

    def config_dc(self):
        return True

    def write_state(self):
        if self.state in (SAFEOP_STATE, OP_STATE) and not self.mapped:
            return
        for s in self.slaves:
            s.state = self.state

    def read_state(self):
        return min((s.state for s in self.slaves), default=INIT_STATE)

    def state_check(self, expected, timeout=50000):
        return self.read_state()

    def send_processdata(self):
        self._wkc = 0
        for s in self.slaves:
            if s.state == OP_STATE and len(s.output) == 28:
                s.input = s.node.pdo(s.output, 0x1A01 in s.tx)
                self.dc_time = s.node.joint.t_ns
                self._wkc += 3
            elif s.state in (SAFEOP_STATE, OP_STATE):
                self._wkc += 1
        return 0

    def receive_processdata(self, timeout=2000):
        return self._wkc

    def close(self):
        pass


def find_adapters():
    return [Adapter("\\Device\\NPF_{FAKE}", "fake NIC")]
