// SPDX-License-Identifier: Apache-2.0
// Runs the ros2_control plugin (src/am_system_interface.cpp) through its lifecycle against the
// mock ROS headers in test/mock_ros. This exercises the plugin logic without ROS; the real-header
// build is the ros2 CI workflow's job.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "am_ros2_control/am_system_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { \
    ++g_checks; \
    if (!(c)) {++g_fail; std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);} \
} while (0)

static hardware_interface::InterfaceInfo itf(const char * n)
{
  hardware_interface::InterfaceInfo i;
  i.name = n;
  return i;
}

static hardware_interface::HardwareInfo make_info(const std::string & backend, const char * extra_cmd = nullptr)
{
  hardware_interface::HardwareInfo info;
  info.name = "am_system";
  info.type = "system";
  info.hardware_parameters["backend"] = backend;
  info.hardware_parameters["cycle_us"] = "1000";
  hardware_interface::ComponentInfo j;
  j.name = "joint1";
  j.type = "joint";
  j.parameters["unit"] = "M";
  j.parameters["sim_load_inertia"] = "0.05";
  for (const char * n : {"position", "velocity", "effort", "stiffness", "damping"}) {
    j.command_interfaces.push_back(itf(n));
  }
  if (extra_cmd) {j.command_interfaces.push_back(itf(extra_cmd));}
  for (const char * n : {"position", "velocity", "effort", "temperature"}) {
    j.state_interfaces.push_back(itf(n));
  }
  info.joints.push_back(j);
  return info;
}

int main()
{
  rclcpp::mock::quiet() = true;
  const rclcpp_lifecycle::State st;
  const rclcpp::Time t;
  const rclcpp::Duration dt(0, 1000000);

  {
    // placeholder backends fail on_init with a clear message
    am_ros2_control::AmSystemInterface hw;
    int before = rclcpp::mock::error_count();
    CHECK(hw.on_init(make_info("ethercat:eth0")) == CallbackReturn::ERROR);
    CHECK(rclcpp::mock::error_count() == before + 1);
  }
  {
    am_ros2_control::AmSystemInterface hw;
    CHECK(hw.on_init(make_info("sim", "current")) == CallbackReturn::ERROR);  // unknown interface
  }
  {
    am_ros2_control::AmSystemInterface hw;
    auto info = make_info("sim");
    info.hardware_parameters["cycle_us"] = "abc";
    CHECK(hw.on_init(info) == CallbackReturn::ERROR);
  }

  am_ros2_control::AmSystemInterface hw;
  CHECK(hw.on_init(make_info("sim")) == CallbackReturn::SUCCESS);
  std::vector<hardware_interface::StateInterface> si = hw.export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> ci = hw.export_command_interfaces();
  CHECK(si.size() == 4 && ci.size() == 5);
  CHECK(si[0].get_name() == "joint1/position" && si[3].get_name() == "joint1/temperature");
  CHECK(ci[3].get_name() == "joint1/stiffness" && ci[4].get_name() == "joint1/damping");

  CHECK(hw.on_configure(st) == CallbackReturn::SUCCESS);
  CHECK(hw.on_activate(st) == CallbackReturn::SUCCESS);
  CHECK(hw.core().drive_state(0) == AM_DS_OPERATION_ENABLED);

  // a position controller starts
  CHECK(hw.prepare_command_mode_switch({"joint1/position"}, {}) == return_type::OK);
  CHECK(hw.perform_command_mode_switch({"joint1/position"}, {}) == return_type::OK);
  ci[0].set_value(0.25);
  bool ok = true;
  for (int k = 0; k < 2000 && ok; ++k) {
    ok = hw.read(t, dt) == return_type::OK && hw.write(t, dt) == return_type::OK;
  }
  CHECK(ok);
  CHECK(std::fabs(si[0].get_value() - 0.25) < 0.01);
  CHECK(std::fabs(si[3].get_value() - 25.0) < 5.0);  // winding temperature, degC

  // switch to impedance with gains from the command interfaces
  CHECK(hw.prepare_command_mode_switch({"joint1/stiffness", "joint1/damping"}, {}) == return_type::OK);
  CHECK(hw.perform_command_mode_switch({"joint1/stiffness", "joint1/damping"}, {}) == return_type::OK);
  CHECK(hw.core().mode(0) == am_ros2_control::ControlMode::kImpedance);
  ci[0].set_value(-0.1);
  ci[3].set_value(400.0);
  ci[4].set_value(12.0);
  for (int k = 0; k < 2000 && ok; ++k) {
    ok = hw.read(t, dt) == return_type::OK && hw.write(t, dt) == return_type::OK;
  }
  CHECK(ok);
  CHECK(std::fabs(si[0].get_value() + 0.1) < 0.02);

  CHECK(hw.prepare_command_mode_switch({"joint1/current"}, {}) == return_type::ERROR);

  // a period that does not match cycle_us warns once
  int warns = rclcpp::mock::warn_count();
  am_ros2_control::AmSystemInterface hw2;
  CHECK(hw2.on_init(make_info("sim")) == CallbackReturn::SUCCESS);
  CHECK(hw2.on_configure(st) == CallbackReturn::SUCCESS);
  for (int k = 0; k < 20; ++k) {hw2.read(t, rclcpp::Duration(0, 10000000));}
  CHECK(rclcpp::mock::warn_count() == warns + 1);
  CHECK(hw2.on_cleanup(st) == CallbackReturn::SUCCESS);

  CHECK(hw.on_deactivate(st) == CallbackReturn::SUCCESS);
  CHECK(hw.core().drive_state(0) == AM_DS_READY_TO_SWITCH_ON);
  CHECK(hw.on_cleanup(st) == CallbackReturn::SUCCESS);
  CHECK(hw.on_configure(st) == CallbackReturn::SUCCESS);
  CHECK(hw.on_activate(st) == CallbackReturn::SUCCESS);
  CHECK(hw.on_error(st) == CallbackReturn::SUCCESS);
  CHECK(hw.core().lifecycle() == am_ros2_control::Lifecycle::kUnconfigured);
  CHECK(hw.on_shutdown(st) == CallbackReturn::SUCCESS);

  std::printf("test_plugin_mock: %d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
