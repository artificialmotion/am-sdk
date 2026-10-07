// Test-only mock: registers nothing. Not ROS.
#pragma once
#define PLUGINLIB_EXPORT_CLASS(class_type, base_class_type) \
  static_assert(sizeof(class_type) > 0 && sizeof(base_class_type) > 0, "pluginlib mock");
