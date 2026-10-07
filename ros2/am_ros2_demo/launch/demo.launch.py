# SPDX-License-Identifier: Apache-2.0
"""One AM joint in ros2_control on the am-sdk simulator. SIMULATED.

    ros2 launch am_ros2_demo demo.launch.py
    ros2 launch am_ros2_demo demo.launch.py unit:=H-X controller:=joint_trajectory_controller

controller: forward_position_controller (default), joint_trajectory_controller, impedance_controller.
Then drive it with `ros2 run am_ros2_demo demo_commands.py --controller <same>`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    unit = LaunchConfiguration("unit")
    backend = LaunchConfiguration("backend")
    controller = LaunchConfiguration("controller")

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("am_ros2_demo"), "urdf", "am_demo_one_joint.urdf.xacro"]),
            " unit:=", unit, " backend:=", backend,
        ]),
        value_type=str)
    controllers = PathJoinSubstitution([FindPackageShare("am_ros2_demo"), "config", "controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("unit", default_value="M", description="S-S, S-D, M, H-F, H-T or H-X"),
        DeclareLaunchArgument("backend", default_value="sim",
                              description="sim (SIMULATED); ethercat/canfd come with evaluation units"),
        DeclareLaunchArgument("controller", default_value="forward_position_controller",
                              description="forward_position_controller, joint_trajectory_controller "
                                          "or impedance_controller"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        # Humble reads robot_description as a parameter; Jazzy also takes it from the topic.
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[{"robot_description": robot_description}, controllers], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=[controller, "--controller-manager", "/controller_manager"]),
    ])
