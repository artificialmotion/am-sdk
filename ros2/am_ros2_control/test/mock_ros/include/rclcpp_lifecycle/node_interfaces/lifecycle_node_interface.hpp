// Test-only mock. Not ROS.
#pragma once
#include <cstdint>
#include "rclcpp_lifecycle/state.hpp"

namespace rclcpp_lifecycle
{
namespace node_interfaces
{
class LifecycleNodeInterface
{
public:
  enum class CallbackReturn : uint8_t { SUCCESS = 97, FAILURE = 98, ERROR = 99 };
  virtual ~LifecycleNodeInterface() = default;
  virtual CallbackReturn on_configure(const State &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_cleanup(const State &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_shutdown(const State &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_activate(const State &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_deactivate(const State &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_error(const State &) {return CallbackReturn::SUCCESS;}
};
}  // namespace node_interfaces
}  // namespace rclcpp_lifecycle
