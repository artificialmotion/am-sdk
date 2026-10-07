# SPDX-License-Identifier: Apache-2.0
"""Expand the AM xacro macros and the demo URDF, and check the limits against the am-sdk unit data.

Needs only the xacro Python package (pip install xacro) and the am-sdk Python package; no ROS.
$(find am_description) is replaced by the source path, so the files are tested as they are in the repo.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip("xacro")

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.dirname(HERE)
ROS2 = os.path.dirname(DESC)
ROOT = os.path.dirname(ROS2)
sys.path.insert(0, os.path.join(ROOT, "python"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from am_sdk import units as U  # noqa: E402


def expand(text, tmp_path, mappings=None):
    text = text.replace("$(find am_description)", DESC.replace("\\", "/"))
    path = tmp_path / "robot.urdf.xacro"
    path.write_text(text, encoding="utf-8")
    doc = xacro.process_file(str(path), mappings=mappings or {})
    return ET.fromstring(doc.toxml())


def robot_for(unit, extra=""):
    return """<?xml version="1.0"?>
<robot name="t" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find am_description)/urdf/am_units.urdf.xacro"/>
  <xacro:include filename="$(find am_description)/urdf/am_ros2_control.xacro"/>
  <link name="a"/><link name="b"/>
  <xacro:am_joint unit="%s" name="j" parent="a" child="b" lower="-1.0" upper="2.0" axis="0 1 0">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </xacro:am_joint>
  <xacro:am_ros2_control name="sys" backend="sim">
    <joints>
      <xacro:am_ros2_control_joint name="j" unit="%s" %s/>
      <xacro:am_ros2_control_joint name="j2" unit="S-D"/>
    </joints>
  </xacro:am_ros2_control>
</robot>""" % (unit, unit, extra)


def test_generated_file_is_current():
    import gen_ros2_xacro
    with open(gen_ros2_xacro.OUT, encoding="utf-8") as fh:
        assert fh.read() == gen_ros2_xacro.generate(), "run python tools/gen_ros2_xacro.py"


@pytest.mark.parametrize("unit", U.UNIT_IDS)
def test_unit_joint_limits(unit, tmp_path):
    ud = U.load_unit(unit)
    root = expand(robot_for(unit), tmp_path)
    j = root.find("joint[@name='j']")
    assert j.get("type") == "revolute"
    lim = j.find("limit")
    assert float(lim.get("effort")) == pytest.approx(ud.peak_nm, rel=1e-5)
    assert float(lim.get("velocity")) == pytest.approx(ud.no_load_rad_s["48"], rel=1e-5)
    assert float(lim.get("lower")) == -1.0 and float(lim.get("upper")) == 2.0
    assert float(j.find("dynamics").get("friction")) == pytest.approx(ud.backdrive_nm, rel=1e-5)
    assert j.find("axis").get("xyz") == "0 1 0"
    assert j.find("origin").get("xyz") == "0 0 0.1"

    rc = root.find("ros2_control")
    assert rc.get("type") == "system"
    assert rc.find("hardware/plugin").text == "am_ros2_control/AmSystemInterface"
    params = {p.get("name"): p.text for p in rc.findall("hardware/param")}
    assert params["backend"] == "sim" and params["cycle_us"] == "1000"
    assert [x.get("name") for x in rc.findall("joint")] == ["j", "j2"]
    rj = rc.find("joint[@name='j']")
    jp = {p.get("name"): p.text for p in rj.findall("param")}
    assert jp["unit"] == unit
    assert "address" not in jp and "default_stiffness" not in jp  # automatic
    cmds = [c.get("name") for c in rj.findall("command_interface")]
    assert cmds == ["position", "velocity", "effort", "stiffness", "damping"]
    states = [s.get("name") for s in rj.findall("state_interface")]
    assert states == ["position", "velocity", "effort", "temperature"]
    eff = rj.find("command_interface[@name='effort']")
    emax = {p.get("name"): float(p.text) for p in eff.findall("param")}
    assert emax["max"] == pytest.approx(ud.peak_nm, rel=1e-5) and emax["min"] == pytest.approx(-ud.peak_nm, rel=1e-5)
    kmax = float(rj.find("command_interface[@name='stiffness']/param[@name='max']").text)
    wn = 2 * math.pi * 60.0
    assert kmax == pytest.approx(ud.armature_kgm2 * wn * wn, rel=1e-5)


def test_explicit_params(tmp_path):
    root = expand(robot_for("M", 'address="0" default_stiffness="250" default_damping="7"'), tmp_path)
    jp = {p.get("name"): p.text for p in root.findall("ros2_control/joint[@name='j']/param")}
    assert jp["address"] == "0" and float(jp["default_stiffness"]) == 250 and float(jp["default_damping"]) == 7


def test_unknown_unit_fails(tmp_path):
    with pytest.raises(Exception):
        expand(robot_for("Q"), tmp_path)


@pytest.mark.parametrize("unit", ["M", "H-X"])
def test_demo_urdf(unit, tmp_path):
    demo = os.path.join(ROS2, "am_ros2_demo", "urdf", "am_demo_one_joint.urdf.xacro")
    with open(demo, encoding="utf-8") as fh:
        root = expand(fh.read(), tmp_path, mappings={"unit": unit})
    assert root.get("name") == "am_demo_one_joint"
    j = root.find("joint[@name='joint1']")
    assert float(j.find("limit").get("effort")) == pytest.approx(U.load_unit(unit).peak_nm, rel=1e-5)
    rj = root.find("ros2_control/joint[@name='joint1']")
    jp = {p.get("name"): p.text for p in rj.findall("param")}
    assert jp["unit"] == unit and float(jp["sim_load_inertia"]) == pytest.approx(0.05)
    assert root.find("ros2_control/hardware/param[@name='backend']").text == "sim"
