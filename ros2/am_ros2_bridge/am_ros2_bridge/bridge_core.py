# SPDX-License-Identifier: Apache-2.0
"""ROS-free core of the am_ros2_bridge node: the am-sdk Python session behind a few plain calls.

The node (bridge_node.py) only moves ROS messages in and out of this class, so everything here is
tested without ROS. Pre-hardware release: the backend is the am-sdk simulator ("sim:<unit>,...").
SIMULATED. "ethercat" and "canfd" raise BridgeError until evaluation units ship.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence

import am_sdk as am

MODES = ("hold", "position", "velocity", "effort", "impedance")
_SDK_MODE = {
    "hold": am.AmMode.CSP,
    "position": am.AmMode.CSP,
    "velocity": am.AmMode.CSV,
    "effort": am.AmMode.CST,
    "impedance": am.AmMode.IMPEDANCE,
}


class BridgeError(RuntimeError):
    pass


def parse_backend(text: str) -> List[str]:
    """'sim:M,H-F' -> ['M', 'H-F']. Raises BridgeError for the hardware placeholders or bad input."""
    t = text.strip()
    kind, _, rest = t.partition(":")
    kind = kind.strip().lower()
    if kind in ("ethercat", "ecat", "canfd", "can"):
        bus = "EtherCAT" if kind in ("ethercat", "ecat") else "CAN-FD"
        raise BridgeError("backend '%s': the %s transport is available with evaluation units (Q1 2027). "
                          "This release runs against the simulator: use sim:<unit>, for example sim:M." % (text, bus))
    if kind != "sim" or not rest.strip():
        raise BridgeError("backend '%s' not recognised; use sim:<unit>[,<unit>...], for example sim:M" % text)
    units = []
    for item in rest.split(","):
        try:
            units.append(am.units.normalize_unit(item))
        except ValueError as e:
            raise BridgeError(str(e)) from None
    return units


class BridgeCore:
    """One am-sdk session; joint i is simulator address i."""

    def __init__(self, backend: str, joint_names: Optional[Sequence[str]] = None, cycle_us: int = 1000,
                 sim_load_inertia: Optional[Sequence[float]] = None):
        self.units = parse_backend(backend)
        n = len(self.units)
        self.joint_names = list(joint_names) if joint_names else ["joint%d" % (i + 1) for i in range(n)]
        if len(self.joint_names) != n:
            raise BridgeError("%d joint name(s) for %d simulated unit(s)" % (len(self.joint_names), n))
        loads = list(sim_load_inertia) if sim_load_inertia else [0.0] * n
        if len(loads) != n:
            raise BridgeError("sim_load_inertia needs one value per joint")
        self.cycle_us = int(cycle_us)
        cfg = am.AmSessionConfig(interface="sim:" + ",".join(self.units), cycle_us=self.cycle_us,
                                 telemetry_extended=True,
                                 sim_options=[am.SimOptions(load_inertia_kgm2=float(x)) for x in loads])
        self.session = am.am_init(cfg)
        self.joints = [am.am_open_joint(self.session, i) for i in range(n)]
        am.am_start(self.session)
        lims = [am.am_get_limits(j)[0] for j in self.joints]
        self.effort_max = [min(lim.torque_pos_Nm, lim.torque_neg_Nm) for lim in lims]
        self.velocity_max = [lim.speed_rad_s for lim in lims]
        self.stiffness_max = [lim.kp_max_Nm_per_rad for lim in lims]
        self.damping_max = [lim.kd_max_Nm_s_per_rad for lim in lims]
        self.mode = "hold"
        self.enabled = False
        self._cmd = [[0.0, 0.0, 0.0] for _ in range(n)]  # per joint, meaning depends on the mode
        self._hold = [0.0] * n
        self.clamped = 0
        self._telemetry = [am.am_read_telemetry(j) for j in self.joints]

    # ------------------------------------------------------------------ lifecycle
    def enable(self, timeout_ms: int = 1000) -> None:
        """Hold the current position (CSP) and walk every joint to Operation enabled."""
        self._refresh()
        self._hold = [t.q_rad for t in self._telemetry]
        self.mode = "hold"
        for j in self.joints:
            am.am_set_mode(j, _SDK_MODE["hold"])
        self._send()
        for j in self.joints:
            am.am_enable(j, timeout_ms=timeout_ms)
        self.enabled = True
        self._refresh()

    def disable(self) -> None:
        for j in self.joints:
            am.am_disable(j)
        for _ in range(10):
            am.am_cycle(self.session)
            self._refresh()
            if all(t.state != am.AmDriveState.OPERATION_ENABLED for t in self._telemetry):
                break
        self.enabled = False

    def close(self) -> None:
        if self.session is not None:
            am.am_deinit(self.session)
            self.session = None

    # ------------------------------------------------------------------ commands
    def _check(self, values: Sequence[float], stride: int, what: str) -> List[float]:
        n = len(self.joints)
        if len(values) != n * stride:
            raise BridgeError("%s needs %d value(s) (%d per joint), got %d" % (what, n * stride, stride, len(values)))
        out = [float(v) for v in values]
        if not all(math.isfinite(v) for v in out):
            raise BridgeError("%s contains NaN or inf" % what)
        return out

    def _switch(self, mode: str) -> None:
        if mode == self.mode:
            return
        self._refresh()
        self._hold = [t.q_rad for t in self._telemetry]
        for j in self.joints:
            am.am_set_mode(j, _SDK_MODE[mode])  # bumpless while Operation enabled
        self.mode = mode

    def command_position(self, q: Sequence[float]) -> None:
        q = self._check(q, 1, "position command")
        self._switch("position")
        for i, v in enumerate(q):
            self._cmd[i] = [v, 0.0, 0.0]

    def command_velocity(self, qd: Sequence[float]) -> None:
        qd = self._check(qd, 1, "velocity command")
        self._switch("velocity")
        for i, v in enumerate(qd):
            self._cmd[i] = [v, 0.0, 0.0]

    def command_effort(self, tau: Sequence[float]) -> None:
        tau = self._check(tau, 1, "effort command")
        self._switch("effort")
        for i, v in enumerate(tau):
            self._cmd[i] = [v, 0.0, 0.0]

    def command_impedance(self, values: Sequence[float]) -> None:
        """[q*, K, D] per joint, the order of MultiInterfaceForwardCommandController [position, stiffness, damping]."""
        v = self._check(values, 3, "impedance command")
        self._switch("impedance")
        for i in range(len(self.joints)):
            q, k, d = v[3 * i:3 * i + 3]
            kc = min(max(k, 0.0), self.stiffness_max[i])
            dc = min(max(d, 0.0), self.damping_max[i])
            if kc != k or dc != d:
                self.clamped += 1
            self._cmd[i] = [q, kc, dc]

    def hold(self) -> None:
        self._switch("hold")

    # ------------------------------------------------------------------ cyclic
    def _send(self) -> None:
        for i, j in enumerate(self.joints):
            c = self._cmd[i]
            if self.mode == "hold":
                am.am_set_position(j, self._hold[i])
            elif self.mode == "position":
                am.am_set_position(j, c[0])
            elif self.mode == "velocity":
                am.am_set_velocity(j, c[0])
            elif self.mode == "effort":
                am.am_set_torque(j, c[0])
            else:
                am.am_set_impedance(j, am.AmImpedance(q_ref_rad=c[0], kp_Nm_per_rad=c[1], kd_Nm_s_per_rad=c[2]))

    def _refresh(self) -> None:
        self._telemetry = [am.am_read_telemetry(j) for j in self.joints]

    def step(self, cycles: int = 1) -> None:
        """Run `cycles` am-sdk cycles with the latest commands. Raises BridgeError when a drive leaves
        Operation enabled while the bridge is enabled (fault, STO, quick stop)."""
        for _ in range(max(1, int(cycles))):
            if self.enabled:
                self._send()
            am.am_cycle(self.session)
            self._refresh()
            if self.enabled:
                for name, t in zip(self.joint_names, self._telemetry):
                    if t.state != am.AmDriveState.OPERATION_ENABLED:
                        self.enabled = False
                        why = ", STO active" if t.sto_active else ""
                        if t.fault_word:
                            bit = (t.fault_word & -t.fault_word).bit_length() - 1
                            why += ", fault %s" % am.am_fault_name(0x1000 + bit)
                        raise BridgeError("joint '%s' left Operation enabled: %s%s" % (name, t.state.name, why))

    def joint_state(self):
        """(names, position rad, velocity rad/s, effort N*m estimated, winding temperature degC)."""
        t = self._telemetry
        return (list(self.joint_names), [x.q_rad for x in t], [x.qd_rad_s for x in t],
                [x.tau_est_Nm for x in t], [x.t_winding_C for x in t])

    def drive_states(self) -> List[str]:
        return [t.state.name for t in self._telemetry]
