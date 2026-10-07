// Test-only mock of hardware_interface::SystemInterface (Humble/Jazzy virtual signatures). Not ROS.
#pragma once
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SystemInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  virtual ~SystemInterface() = default;
  virtual CallbackReturn on_init(const HardwareInfo & hardware_info)
  {
    info_ = hardware_info;
    return CallbackReturn::SUCCESS;
  }
  virtual std::vector<StateInterface> export_state_interfaces() = 0;
  virtual std::vector<CommandInterface> export_command_interfaces() = 0;
  virtual return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) {return return_type::OK;}
  virtual return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) {return return_type::OK;}
  virtual return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
  virtual return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

protected:
  HardwareInfo info_;
};
}  // namespace hardware_interface
