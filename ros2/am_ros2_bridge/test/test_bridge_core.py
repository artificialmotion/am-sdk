# SPDX-License-Identifier: Apache-2.0
"""Tests of the ROS-free bridge core against the am-sdk Python simulator. No ROS needed."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
am = pytest.importorskip("am_sdk")

from am_ros2_bridge.bridge_core import BridgeCore, BridgeError, parse_backend  # noqa: E402


def test_parse_backend():
    assert parse_backend("sim:M") == ["M"]
    assert parse_backend(" sim: m , h_f ") == ["M", "H-F"]
    for bad in ("sim", "sim:", "sim:Q", "serial:x"):
        with pytest.raises(BridgeError):
            parse_backend(bad)
    for hw in ("ethercat:eth0", "canfd:can0"):
        with pytest.raises(BridgeError, match="available with evaluation units"):
            parse_backend(hw)


def test_name_count_mismatch():
    with pytest.raises(BridgeError):
        BridgeCore("sim:M,M", ["only_one"])


def test_modes_track():
    b = BridgeCore("sim:M", ["knee"], sim_load_inertia=[0.05])
    try:
        assert b.effort_max[0] == pytest.approx(76.5)
        assert b.velocity_max[0] == pytest.approx(12.3674, abs=1e-3)
        b.enable()
        assert b.drive_states() == ["OPERATION_ENABLED"]
        b.step(200)
        assert abs(b.joint_state()[1][0]) < 0.01

        b.command_position([0.3])
        b.step(2000)
        names, q, qd, tau, temp = b.joint_state()
        assert names == ["knee"]
        assert q[0] == pytest.approx(0.3, abs=0.01)
        assert temp[0] == pytest.approx(25.0, abs=1.0)

        b.command_velocity([1.0])
        b.step(1000)
        assert b.joint_state()[2][0] == pytest.approx(1.0, abs=0.1)

        b.command_effort([0.0])
        b.step(1500)
        v0 = b.joint_state()[2][0]
        b.command_effort([5.0])
        b.step(100)
        assert b.joint_state()[2][0] > v0 + 1.0
        assert b.joint_state()[3][0] == pytest.approx(5.0, abs=1.0)

        b.command_impedance([0.2, 300.0, 10.0])
        b.step(3000)
        assert b.joint_state()[1][0] == pytest.approx(0.2, abs=0.02)
        b.command_impedance([0.2, 1e9, 10.0])  # above the K cap: clamped, counted
        assert b.clamped == 1
        b.step(10)

        with pytest.raises(BridgeError):
            b.command_position([0.1, 0.2])
        with pytest.raises(BridgeError):
            b.command_position([math.nan])

        b.hold()
        b.step(10)
        b.disable()
        assert b.drive_states() == ["READY_TO_SWITCH_ON"]
    finally:
        b.close()


def test_fault_stops_bridge():
    b = BridgeCore("sim:H-F")
    try:
        b.enable()
        b.step(5)
        b.joints[0].jb.inject_fault(0x1001)  # the SimJoint behind the session joint
        with pytest.raises(BridgeError, match="left Operation enabled"):
            b.step(20)
        assert not b.enabled
    finally:
        b.close()
