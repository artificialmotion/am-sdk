// SPDX-License-Identifier: Apache-2.0
#include "am_ros2_control/am_system_interface.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace am_ros2_control
{

namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

rclcpp::Logger logger() {return rclcpp::get_logger("AmSystemInterface");}

template<typename Map>
bool get_param(const Map & params, const std::string & key, std::string * out)
{
  auto it = params.find(key);
  if (it == params.end()) {return false;}
  *out = it->second;
  return true;
}

template<typename Map>
bool get_double(const Map & params, const std::string & key, double * out, std::string * error)
{
  std::string s;
  if (!get_param(params, key, &s)) {return true;}  // keep the default
  char * end = nullptr;
  errno = 0;
  double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0' || errno != 0 || !std::isfinite(v)) {
    *error = "parameter '" + key + "' = '" + s + "' is not a finite number";
    return false;
  }
  *out = v;
  return true;
}

template<typename Map>
bool get_uint(const Map & params, const std::string & key, unsigned long * out, std::string * error)  // NOLINT
{
  double v = static_cast<double>(*out);
  if (!get_double(params, key, &v, error)) {return false;}
  if (v < 0 || v != std::floor(v) || v > 4294967295.0) {
    *error = "parameter '" + key + "' must be a non-negative integer";
    return false;
  }
  *out = static_cast<unsigned long>(v);  // NOLINT
  return true;
}

bool known_state(const std::string & n)
{
  return n == kIfPosition || n == kIfVelocity || n == kIfEffort || n == kIfTemperature;
}

bool known_command(const std::string & n)
{
  return n == kIfPosition || n == kIfVelocity || n == kIfEffort || n == kIfStiffness || n == kIfDamping;
}

}  // namespace

CallbackReturn AmSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  std::string error;
  std::string backend = "sim";
  get_param(info_.hardware_parameters, "backend", &backend);
  unsigned long cycle_us = 1000, enable_timeout_ms = 1000;  // NOLINT
  if (!get_uint(info_.hardware_parameters, "cycle_us", &cycle_us, &error) ||
    !get_uint(info_.hardware_parameters, "enable_timeout_ms", &enable_timeout_ms, &error))
  {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }

  std::vector<JointConfig> joints;
  for (const auto & j : info_.joints) {
    JointConfig c;
    c.name = j.name;
    get_param(j.parameters, "unit", &c.unit);
    double addr = -1.0;
    if (!get_double(j.parameters, "address", &addr, &error) ||
      !get_double(j.parameters, "default_stiffness", &c.default_stiffness, &error) ||
      !get_double(j.parameters, "default_damping", &c.default_damping, &error) ||
      !get_double(j.parameters, "sim_load_inertia", &c.sim_load_inertia, &error) ||
      !get_double(j.parameters, "sim_initial_position", &c.sim_initial_position, &error))
    {
      RCLCPP_ERROR(logger(), "joint '%s': %s", j.name.c_str(), error.c_str());
      return CallbackReturn::ERROR;
    }
    if (addr != std::floor(addr) || addr > 65535.0) {
      RCLCPP_ERROR(logger(), "joint '%s': address must be an integer", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    c.address = static_cast<int>(addr);
    for (const auto & s : j.state_interfaces) {
      if (!known_state(s.name)) {
        RCLCPP_ERROR(
          logger(), "joint '%s': state interface '%s' not provided; use position, velocity, effort, "
          "temperature", j.name.c_str(), s.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    for (const auto & ci : j.command_interfaces) {
      if (!known_command(ci.name)) {
        RCLCPP_ERROR(
          logger(), "joint '%s': command interface '%s' not provided; use position, velocity, effort, "
          "stiffness, damping", j.name.c_str(), ci.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    joints.push_back(c);
  }
  if (!core_.init(
      backend, static_cast<uint32_t>(cycle_us), static_cast<uint32_t>(enable_timeout_ms), joints, &error))
  {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger(), "am_ros2_control: %u joint(s) on backend '%s' (SIMULATED), cycle %u us",
    static_cast<unsigned>(joints.size()), backend.c_str(), static_cast<unsigned>(cycle_us));
  return CallbackReturn::SUCCESS;
}

CallbackReturn AmSystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!core_.configure(&error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  for (std::size_t j = 0; j < core_.size(); ++j) {
    const JointLimitsInfo & l = core_.limits(j);
    RCLCPP_INFO(
      logger(), "joint '%s': effort <= %.1f N*m, velocity <= %.2f rad/s, K <= %.0f, D <= %.1f",
      core_.joint_name(j).c_str(), l.effort_max, l.velocity_max, l.stiffness_max, l.damping_max);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AmSystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  core_.cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AmSystemInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!core_.deactivate(&error)) {RCLCPP_WARN(logger(), "%s", error.c_str());}
  core_.cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AmSystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!core_.activate(&error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "joints in Operation enabled (SIMULATED)");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AmSystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!core_.deactivate(&error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AmSystemInterface::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(logger(), "hardware error: quick stop and close the session");
  core_.emergency_stop();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AmSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t j = 0; j < info_.joints.size(); ++j) {
    const auto & ji = info_.joints[j];
    JointState & s = core_.state(j);
    for (const auto & si : ji.state_interfaces) {
      double * p = nullptr;
      if (si.name == kIfPosition) {p = &s.position;} else if (si.name == kIfVelocity) {
        p = &s.velocity;
      } else if (si.name == kIfEffort) {p = &s.effort;} else if (si.name == kIfTemperature) {
        p = &s.temperature;
      }
      if (p) {out.emplace_back(ji.name, si.name, p);}
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> AmSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (std::size_t j = 0; j < info_.joints.size(); ++j) {
    const auto & ji = info_.joints[j];
    JointCommand & c = core_.command(j);
    for (const auto & ci : ji.command_interfaces) {
      double * p = nullptr;
      if (ci.name == kIfPosition) {p = &c.position;} else if (ci.name == kIfVelocity) {
        p = &c.velocity;
      } else if (ci.name == kIfEffort) {p = &c.effort;} else if (ci.name == kIfStiffness) {
        p = &c.stiffness;
      } else if (ci.name == kIfDamping) {p = &c.damping;}
      if (p) {out.emplace_back(ji.name, ci.name, p);}
    }
  }
  return out;
}

return_type AmSystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::string error;
  planned_valid_ = core_.plan_switch(start_interfaces, stop_interfaces, &planned_, &error);
  if (!planned_valid_) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type AmSystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::string error;
  if (!planned_valid_ &&
    !core_.plan_switch(start_interfaces, stop_interfaces, &planned_, &error))
  {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return return_type::ERROR;
  }
  planned_valid_ = false;
  if (!core_.apply_switch(planned_, &error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return return_type::ERROR;
  }
  for (std::size_t j = 0; j < core_.size(); ++j) {
    RCLCPP_INFO(
      logger(), "joint '%s': mode %s", core_.joint_name(j).c_str(),
      control_mode_name(core_.mode(j)));
  }
  return return_type::OK;
}

return_type AmSystemInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // One write() advances the simulator by one cycle_us; warn once when the controller manager
  // runs at another rate (simulated time then drifts from ROS time).
  if (!period_warned_ && ++period_checks_ > 10) {
    const double expected = core_.cycle_us() * 1e-6;
    const double p = period.seconds();
    if (p > 0.0 && std::fabs(p - expected) > 0.25 * expected) {
      RCLCPP_WARN(
        logger(), "controller period %.4f s differs from cycle_us %u; set the hardware parameter "
        "cycle_us to 1e6 / update_rate", p, core_.cycle_us());
    }
    period_warned_ = true;
  }
  std::string error;
  if (!core_.read(&error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type AmSystemInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string error;
  if (!core_.write(&error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return return_type::ERROR;
  }
  if (core_.clamp_count() != clamps_reported_ &&
    (clamps_reported_ == 0 || core_.clamp_count() - clamps_reported_ >= 1000))
  {
    RCLCPP_WARN(
      logger(), "stiffness/damping command above the drive cap; clamped (%lu cycles so far)",
      static_cast<unsigned long>(core_.clamp_count()));  // NOLINT
    clamps_reported_ = core_.clamp_count();
  }
  return return_type::OK;
}

}  // namespace am_ros2_control

PLUGINLIB_EXPORT_CLASS(am_ros2_control::AmSystemInterface, hardware_interface::SystemInterface)
