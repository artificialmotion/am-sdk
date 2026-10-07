// SPDX-License-Identifier: Apache-2.0
// ROS 2 test (colcon test): load the plugin through pluginlib, parse a real ros2_control URDF
// block and run the lifecycle against the simulator. Needs ROS 2; runs in the ros2 CI workflow.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{
const char kUrdf[] = R"(<?xml version="1.0"?>
<robot name="am_test">
  <link name="world"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/><child link="link1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="80" velocity="12.6"/>
  </joint>
  <ros2_control name="am_system" type="system">
    <hardware>
      <plugin>am_ros2_control/AmSystemInterface</plugin>
      <param name="backend">sim:M</param>
      <param name="cycle_us">1000</param>
    </hardware>
    <joint name="joint1">
      <param name="sim_load_inertia">0.05</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <command_interface name="stiffness"/>
      <command_interface name="damping"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <state_interface name="temperature"/>
    </joint>
  </ros2_control>
</robot>)";
}  // namespace

TEST(AmSystemInterface, LoadsThroughPluginlibAndTracksPosition)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  std::unique_ptr<hardware_interface::SystemInterface> hw;
  ASSERT_NO_THROW(
    hw = std::unique_ptr<hardware_interface::SystemInterface>(
      loader.createUnmanagedInstance("am_ros2_control/AmSystemInterface")));
  ASSERT_TRUE(hw);

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(kUrdf);
  ASSERT_EQ(infos.size(), 1u);
  ASSERT_EQ(hw->on_init(infos[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto states = hw->export_state_interfaces();
  auto commands = hw->export_command_interfaces();
  ASSERT_EQ(states.size(), 4u);
  ASSERT_EQ(commands.size(), 5u);

  const rclcpp_lifecycle::State st;
  ASSERT_EQ(hw->on_configure(st), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(st), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hw->prepare_command_mode_switch({"joint1/position"}, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(
    hw->perform_command_mode_switch({"joint1/position"}, {}), hardware_interface::return_type::OK);

  for (auto & c : commands) {
    if (c.get_name() == "joint1/position") {
      c.set_value(0.25);
    }
  }
  const rclcpp::Time t(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt(0, 1000000);
  for (int k = 0; k < 2000; ++k) {
    ASSERT_EQ(hw->read(t, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(hw->write(t, dt), hardware_interface::return_type::OK);
  }
  double q = std::nan("");
  for (auto & s : states) {
    if (s.get_name() == "joint1/position") {
      q = s.get_value();
    }
  }
  EXPECT_NEAR(q, 0.25, 0.01);
  EXPECT_EQ(hw->on_deactivate(st), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hw->on_cleanup(st), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(AmSystemInterface, HardwareBackendsAreNotAvailableYet)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  std::unique_ptr<hardware_interface::SystemInterface> hw(
    loader.createUnmanagedInstance("am_ros2_control/AmSystemInterface"));
  std::string urdf(kUrdf);
  urdf.replace(urdf.find("sim:M"), 5, "ethercat:eth0");
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(hw->on_init(infos[0]), hardware_interface::CallbackReturn::ERROR);
}
