# Mock ROS 2 headers (test only)

A minimal stand-in for the parts of `hardware_interface` (ros2_control Humble/Jazzy API), `rclcpp`,
`rclcpp_lifecycle` and `pluginlib` that `src/am_system_interface.cpp` uses. They let the plugin compile
and run its lifecycle in a plain CMake build on machines without ROS 2.

They are not ROS. Passing against them shows the plugin logic works. It does not show that the plugin
builds against the real headers: the `ros2` CI workflow (ros:humble and ros:jazzy containers) checks that.
