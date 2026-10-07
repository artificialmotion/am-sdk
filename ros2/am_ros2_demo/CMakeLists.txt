# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(am_ros2_demo NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/demo_commands.py DESTINATION lib/${PROJECT_NAME})

ament_package()
