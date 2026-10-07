#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Send demo commands to the controller started by demo.launch.py. SIMULATED.

    ros2 run am_ros2_demo demo_commands.py --controller forward_position_controller
    ros2 run am_ros2_demo demo_commands.py --controller impedance_controller
    ros2 run am_ros2_demo demo_commands.py --controller joint_trajectory_controller
"""
import argparse
import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

AMP, FREQ = 0.5, 0.5  # rad, Hz


class Sine(Node):
    def __init__(self, controller, stiffness, damping):
        super().__init__("am_demo_commands")
        self.controller, self.k, self.d = controller, stiffness, damping
        self.pub = self.create_publisher(Float64MultiArray, "/%s/commands" % controller, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        q = AMP * math.sin(2.0 * math.pi * FREQ * t)
        msg = Float64MultiArray()
        msg.data = [q] if self.controller == "forward_position_controller" else [q, self.k, self.d]
        self.pub.publish(msg)


def send_trajectory(node, joint):
    client = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("joint_trajectory_controller action server not available")
        return
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [joint]
    for i, (q, t) in enumerate(((0.8, 1.0), (-0.8, 3.0), (0.0, 4.5))):
        p = JointTrajectoryPoint()
        p.positions = [q]
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        goal.trajectory.points.append(p)
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle or not handle.accepted:
        node.get_logger().error("goal rejected")
        return
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    node.get_logger().info("trajectory done, error_code %d" % result.result().result.error_code)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default="forward_position_controller",
                    choices=["forward_position_controller", "impedance_controller", "joint_trajectory_controller"])
    ap.add_argument("--joint", default="joint1")
    ap.add_argument("--stiffness", type=float, default=150.0, help="impedance K, N*m/rad")
    ap.add_argument("--damping", type=float, default=6.0, help="impedance D, N*m*s/rad")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    if args.controller == "joint_trajectory_controller":
        node = Node("am_demo_commands")
        send_trajectory(node, args.joint)
    else:
        node = Sine(args.controller, args.stiffness, args.damping)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
