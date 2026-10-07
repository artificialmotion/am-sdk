// Test-only mock. Not ROS.
#pragma once
#include <cstdint>
#include <string>

namespace rclcpp_lifecycle
{
class State
{
public:
  State()
  : id_(0), label_("unknown") {}
  State(uint8_t id, const std::string & label)
  : id_(id), label_(label) {}
  uint8_t id() const {return id_;}
  const std::string & label() const {return label_;}

private:
  uint8_t id_;
  std::string label_;
};
}  // namespace rclcpp_lifecycle
