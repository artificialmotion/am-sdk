// Test-only mock. Not ROS.
#pragma once
#include <cstdint>

namespace hardware_interface
{
enum class return_type : std::uint8_t { OK = 0, ERROR = 1 };
}  // namespace hardware_interface
