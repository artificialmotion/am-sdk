# SPDX-License-Identifier: Apache-2.0
#
# am_ros2_control: ros2_control SystemInterface plugin for Artificial Motion joints.
#
# Two ways to build:
#   colcon (ROS 2 Humble/Jazzy):  colcon build --base-paths ros2      -> plugin + ROS tests
#   standalone (no ROS):          cmake -S ros2/am_ros2_control -B build -DAM_ROS2_STANDALONE=ON
#                                 -> ROS-free core, core tests, plugin tests on mock ROS headers
cmake_minimum_required(VERSION 3.16)
project(am_ros2_control VERSION 0.1.0 LANGUAGES C CXX)

option(AM_ROS2_STANDALONE "Build the ROS-free core and the mock-header tests without ament" OFF)
set(AM_SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../c" CACHE PATH "am-sdk C library (the repo's c/ folder)")

if(NOT EXISTS "${AM_SDK_SOURCE_DIR}/include/am_api.h")
  message(FATAL_ERROR "am-sdk C sources not found at AM_SDK_SOURCE_DIR=${AM_SDK_SOURCE_DIR}. "
                      "Build from the am-sdk repository (colcon build --base-paths ros2) or set AM_SDK_SOURCE_DIR.")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # the static core links into the plugin .so

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(AM_ROS2_WARN -Wall -Wextra -pedantic)
elseif(MSVC)
  set(AM_ROS2_WARN /W4 /D_CRT_SECURE_NO_WARNINGS)
endif()
option(AM_ROS2_WERROR "Treat warnings as errors in the core and the plugin" OFF)
if(AM_ROS2_WERROR)
  if(MSVC)
    list(APPEND AM_ROS2_WARN /WX)
  else()
    list(APPEND AM_ROS2_WARN -Werror)
  endif()
endif()

# am-sdk C library, compiled in (static, PIC). Nothing of it is installed.
add_library(am_ros2_sdk STATIC
  ${AM_SDK_SOURCE_DIR}/src/am_codec.c
  ${AM_SDK_SOURCE_DIR}/src/am_session.c
  ${AM_SDK_SOURCE_DIR}/src/am_sim.c
  ${AM_SDK_SOURCE_DIR}/src/am_unit_data.c)
target_include_directories(am_ros2_sdk PUBLIC $<BUILD_INTERFACE:${AM_SDK_SOURCE_DIR}/include>)
if(MSVC)
  target_compile_definitions(am_ros2_sdk PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
if(NOT WIN32)
  target_link_libraries(am_ros2_sdk PUBLIC m)
endif()

# ROS-free core: backend parsing, lifecycle -> CiA 402, claims -> mode, cyclic read/write.
add_library(am_ros2_control_core STATIC src/am_system_core.cpp)
target_include_directories(am_ros2_control_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(am_ros2_control_core PUBLIC am_ros2_sdk)
target_compile_options(am_ros2_control_core PRIVATE ${AM_ROS2_WARN})

if(AM_ROS2_STANDALONE)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE am_ros2_control_core)
  target_compile_options(test_core PRIVATE ${AM_ROS2_WARN})
  add_test(NAME test_core COMMAND test_core)

  # The plugin source itself, compiled against test/mock_ros (not ROS; see its README).
  add_executable(test_plugin_mock test/test_plugin_mock.cpp src/am_system_interface.cpp)
  target_include_directories(test_plugin_mock PRIVATE test/mock_ros/include)
  target_link_libraries(test_plugin_mock PRIVATE am_ros2_control_core)
  target_compile_options(test_plugin_mock PRIVATE ${AM_ROS2_WARN})
  add_test(NAME test_plugin_mock COMMAND test_plugin_mock)
  return()
endif()

# ------------------------------------------------------------------ ROS 2 build (ament)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(am_ros2_control SHARED src/am_system_interface.cpp)
target_include_directories(am_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/am_ros2_control>)
target_link_libraries(am_ros2_control PRIVATE am_ros2_control_core)
target_compile_options(am_ros2_control PRIVATE ${AM_ROS2_WARN})
ament_target_dependencies(am_ros2_control hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface am_ros2_control.xml)

install(TARGETS am_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/am_ros2_control)

if(BUILD_TESTING)
  # ROS-free core test, same as the standalone build.
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE am_ros2_control_core)
  add_test(NAME test_core COMMAND test_core)

  # Loads the installed plugin through pluginlib and runs it against the simulator.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)
  ament_add_gtest(test_load_am_system test/test_load_am_system.cpp)
  if(TARGET test_load_am_system)
    target_include_directories(test_load_am_system PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(test_load_am_system hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  endif()
endif()

ament_export_include_directories(include/am_ros2_control)
ament_export_libraries(am_ros2_control)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
