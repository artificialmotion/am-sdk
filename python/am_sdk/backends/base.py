# SPDX-License-Identifier: Apache-2.0
"""Pluggable backend interface.

A Backend owns the transport (sim plant, EtherCAT, CAN-FD). The SDK layer (am_sdk.api)
owns the handle model, mode rules and the enable sequence, and talks to a backend only
through the two classes below. Implement them to add a transport, or implement the smaller
`Transport` interface in backends/transport.py and let TransportBackend do the rest.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

from ..am_types import (AmCommand, AmDeviceInfo, AmFaultInfo, AmFaultRecord, AmIdentity, AmLimits,
                        AmLoopGains, AmMode, AmSessionConfig, AmTelemetry, AmTransparency)

# one cyclic request: (joint backend, controlword, mode, command, command is new this cycle)
CycleItem = Tuple["JointBackend", int, AmMode, AmCommand, bool]


class JointBackend:
    """One actuator behind a backend."""

    def identity(self) -> AmIdentity:
        raise NotImplementedError

    def transparency(self) -> AmTransparency:
        raise NotImplementedError

    def exchange(self, controlword: int, mode: AmMode, cmd: AmCommand, cmd_fresh: bool) -> AmTelemetry:
        """Send one cycle of process data and return the feedback of that cycle."""
        raise NotImplementedError

    def fault_info(self) -> AmFaultInfo:
        raise NotImplementedError

    def fault_history(self) -> List[AmFaultRecord]:
        raise NotImplementedError

    def get_limits(self) -> Tuple[AmLimits, AmLimits]:
        """(effective, ceiling)"""
        raise NotImplementedError

    def set_limits(self, lim: AmLimits) -> None:
        raise NotImplementedError

    def get_loop_gains(self) -> AmLoopGains:
        raise NotImplementedError

    def set_loop_gains(self, g: AmLoopGains) -> None:
        raise NotImplementedError

    def sdo_read(self, index: int, sub: int) -> bytes:
        raise NotImplementedError

    def sdo_write(self, index: int, sub: int, data: bytes) -> None:
        raise NotImplementedError

    def store_parameters(self) -> None:
        raise NotImplementedError

    def restore_defaults(self) -> None:
        raise NotImplementedError

    def set_user_label(self, label: str) -> None:
        raise NotImplementedError

    def get_control_bus(self) -> Tuple[int, int]:
        raise NotImplementedError

    def set_control_bus(self, bus: int) -> None:
        raise NotImplementedError

    def read_calibration(self) -> str:
        raise NotImplementedError

    def pop_events(self) -> List[Tuple[int, int, int]]:
        """Drain (event_kind, code, t_ns) tuples raised since the last call."""
        return []

    def comms_lost(self) -> None:
        """Host stopped the cyclic exchange: drive reaction (zero torque, then disarm)."""

    def last_abort_code(self) -> int:
        return 0


class Backend:
    """A bus connection."""

    name = "base"

    def open(self, cfg: AmSessionConfig) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def scan(self) -> List[AmDeviceInfo]:
        raise NotImplementedError

    def open_joint(self, address: int) -> JointBackend:
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def cycle(self, items: Sequence[CycleItem]) -> List[AmTelemetry]:
        """One cycle for every opened joint, in open order. The default calls exchange() per joint;
        bus backends override it to send one frame set for all joints."""
        return [jb.exchange(cw, mode, cmd, fresh) for jb, cw, mode, cmd, fresh in items]
