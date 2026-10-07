# SPDX-License-Identifier: Apache-2.0
"""Start am_bridge for one simulated joint. SIMULATED.

    ros2 launch am_ros2_bridge bridge.launch.py backend:=sim:M
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="sim:M",
                              description="sim:<unit>[,<unit>...] (SIMULATED)"),
        Node(package="am_ros2_bridge", executable="am_bridge", name="am_bridge", output="screen",
             parameters=[{"backend": LaunchConfiguration("backend"),
                          "joint_names": ["joint1"],
                          "sim_load_inertia": [0.05]}]),
    ])
