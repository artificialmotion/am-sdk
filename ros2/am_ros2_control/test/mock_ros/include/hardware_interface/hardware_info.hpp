// Test-only mock of the ros2_control HardwareInfo structs (Humble/Jazzy field names). Not ROS.
#pragma once
#include <string>
#include <unordered_map>
#include <vector>

namespace hardware_interface
{
struct InterfaceInfo
{
  std::string name;
  std::string min;
  std::string max;
  std::string initial_value;
  std::string data_type;
  int size = 0;
};

struct ComponentInfo
{
  std::string name;
  std::string type;
  std::vector<InterfaceInfo> command_interfaces;
  std::vector<InterfaceInfo> state_interfaces;
  std::unordered_map<std::string, std::string> parameters;
};

struct HardwareInfo
{
  std::string name;
  std::string type;
  std::unordered_map<std::string, std::string> hardware_parameters;
  std::vector<ComponentInfo> joints;
  std::vector<ComponentInfo> sensors;
  std::vector<ComponentInfo> gpios;
};
}  // namespace hardware_interface
