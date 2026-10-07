// Test-only mock of hardware_interface::StateInterface / CommandInterface. Not ROS.
#pragma once
#include <string>

namespace hardware_interface
{
class Handle
{
public:
  Handle(const std::string & prefix_name, const std::string & interface_name, double * value_ptr = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr) {}
  std::string get_name() const {return prefix_name_ + "/" + interface_name_;}
  const std::string & get_interface_name() const {return interface_name_;}
  const std::string & get_prefix_name() const {return prefix_name_;}
  double get_value() const {return *value_ptr_;}

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double * value_ptr_;
};

class StateInterface : public Handle
{
public:
  using Handle::Handle;
};

class CommandInterface : public Handle
{
public:
  using Handle::Handle;
  void set_value(double v) {*value_ptr_ = v;}
};
}  // namespace hardware_interface
