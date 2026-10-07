// SPDX-License-Identifier: Apache-2.0
//
// am_system_interface.hpp - ros2_control SystemInterface for Artificial Motion joints.
//
// Plugin name: am_ros2_control/AmSystemInterface (see am_ros2_control.xml).
//
// Hardware parameters (<hardware><param name="...">):
//   backend            "sim:<unit>[,<unit>...]" (e.g. sim:M), or "sim" with a unit per joint.
//                      "ethercat:<nic>" / "canfd:<if>" are placeholders: on_init fails with
//                      "available with evaluation units" until the hardware transport ships.
//   cycle_us           am-sdk cycle; one write() runs one cycle. Match the controller_manager
//                      update_rate (1000 us <-> 1000 Hz). Default 1000.
//   enable_timeout_ms  CiA 402 enable walk timeout. Default 1000.
// Joint parameters (<joint><param name="...">):
//   unit               S-S, S-D, M, H-F, H-T, H-X (required with backend "sim")
//   address            simulator address; default = joint index
//   default_stiffness  impedance K when "stiffness" is not claimed, N*m/rad
//                      (default: 5 % of the unit's K cap, e.g. 213 for M)
//   default_damping    impedance D when "damping" is not claimed, N*m*s/rad
//                      (default: 25 % of the unit's D cap, e.g. 5.7 for M)
//   sim_load_inertia   simulator: link inertia on the output, kg*m^2 (default 0)
//   sim_initial_position  simulator: initial output angle, rad (default 0)
//
// State interfaces: position, velocity, effort (estimated output torque), temperature (winding).
// Command interfaces: position, velocity, effort, stiffness, damping.
// Drive mode follows the claimed command interfaces (am_system_core.hpp, ControlMode).
//
// Lifecycle -> CiA 402 (through the am-sdk session API):
//   on_init        validate parameters (no I/O)
//   on_configure   am_init + am_open_joint + am_start   -> Switch on disabled
//   on_activate    am_set_mode + am_enable                -> Operation enabled
//   on_deactivate  am_disable                             -> Ready to switch on
//   on_cleanup     am_deinit
//   on_shutdown    am_deinit (disables first)
//   on_error       am_quick_stop, then am_deinit
#ifndef AM_ROS2_CONTROL__AM_SYSTEM_INTERFACE_HPP_
#define AM_ROS2_CONTROL__AM_SYSTEM_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "am_ros2_control/am_system_core.hpp"

namespace am_ros2_control
{

class AmSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // The ROS-free core (tests, diagnostics).
  AmSystemCore & core() {return core_;}

private:
  AmSystemCore core_;
  std::vector<Claims> planned_;
  bool planned_valid_ = false;
  unsigned period_checks_ = 0;
  bool period_warned_ = false;
  uint64_t clamps_reported_ = 0;
};

}  // namespace am_ros2_control

#endif  // AM_ROS2_CONTROL__AM_SYSTEM_INTERFACE_HPP_
