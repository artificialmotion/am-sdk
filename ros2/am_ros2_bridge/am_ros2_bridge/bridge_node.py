# SPDX-License-Identifier: Apache-2.0
"""am_bridge: a Python-only ROS 2 node for AM joints on the am-sdk simulator. SIMULATED.

The fallback when you do not run ros2_control. Publishes sensor_msgs/JointState, takes commands as
std_msgs/Float64MultiArray (one value per joint, in joint_names order):

    joint_states                 sensor_msgs/JointState   position rad, velocity rad/s, effort N*m (estimated)
    ~/temperatures               Float64MultiArray        winding temperature, degC
    ~/position_commands          Float64MultiArray        -> CSP
    ~/velocity_commands          Float64MultiArray        -> CSV
    ~/effort_commands            Float64MultiArray        -> CST
    ~/impedance_commands         Float64MultiArray        [q*, K, D] per joint -> AM impedance mode
    ~/enable, ~/disable, ~/hold  std_srvs/Trigger

Parameters: backend ("sim:M"), joint_names, cycle_us (1000), publish_rate (100 Hz), auto_enable (true),
sim_load_inertia (one value per joint, kg*m^2). Each timer tick runs 1e6 / (publish_rate * cycle_us)
simulator cycles, so simulated time follows wall time.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from am_ros2_bridge.bridge_core import BridgeCore, BridgeError


class AmBridgeNode(Node):
    def __init__(self):
        super().__init__("am_bridge")
        backend = self.declare_parameter("backend", "sim:M").value
        names = list(self.declare_parameter("joint_names", ["joint1"]).value)
        cycle_us = int(self.declare_parameter("cycle_us", 1000).value)
        rate = float(self.declare_parameter("publish_rate", 100.0).value)
        auto_enable = bool(self.declare_parameter("auto_enable", True).value)
        loads = list(self.declare_parameter("sim_load_inertia", [0.0]).value)
        if len(loads) == 1 and len(names) > 1:
            loads = loads * len(names)
        self.core = BridgeCore(backend, names, cycle_us, loads)  # raises BridgeError with a clear message
        self.cycles_per_tick = max(1, int(round(1e6 / (rate * cycle_us))))
        self.get_logger().info("am_bridge: %s on %s (SIMULATED), %d cycle(s) of %d us per tick"
                               % (", ".join(names), backend, self.cycles_per_tick, cycle_us))

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_temp = self.create_publisher(Float64MultiArray, "~/temperatures", 10)
        self.create_subscription(Float64MultiArray, "~/position_commands",
                                 lambda m: self._cmd(self.core.command_position, m), 10)
        self.create_subscription(Float64MultiArray, "~/velocity_commands",
                                 lambda m: self._cmd(self.core.command_velocity, m), 10)
        self.create_subscription(Float64MultiArray, "~/effort_commands",
                                 lambda m: self._cmd(self.core.command_effort, m), 10)
        self.create_subscription(Float64MultiArray, "~/impedance_commands",
                                 lambda m: self._cmd(self.core.command_impedance, m), 10)
        self.create_service(Trigger, "~/enable", self._srv(self._enable))
        self.create_service(Trigger, "~/disable", self._srv(self.core.disable))
        self.create_service(Trigger, "~/hold", self._srv(self.core.hold))
        if auto_enable:
            self._enable()
        self.create_timer(1.0 / rate, self._tick)

    def _enable(self):
        self.core.enable()
        self.get_logger().info("joints in Operation enabled, holding position (SIMULATED)")

    def _cmd(self, fn, msg):
        if not self.core.enabled:
            self.get_logger().warn("command ignored: joints not enabled (call ~/enable)", throttle_duration_sec=2.0)
            return
        try:
            fn(msg.data)
        except BridgeError as e:
            self.get_logger().error(str(e), throttle_duration_sec=1.0)

    def _srv(self, fn):
        def handler(_req, resp):
            try:
                fn()
                resp.success, resp.message = True, ", ".join(self.core.drive_states())
            except Exception as e:  # noqa: BLE001 - report SDK errors to the caller
                resp.success, resp.message = False, str(e)
            return resp
        return handler

    def _tick(self):
        try:
            self.core.step(self.cycles_per_tick)
        except BridgeError as e:
            self.get_logger().error(str(e))
        names, q, qd, tau, temp = self.core.joint_state()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position, js.velocity, js.effort = names, q, qd, tau
        self.pub_js.publish(js)
        self.pub_temp.publish(Float64MultiArray(data=temp))

    def destroy_node(self):
        self.core.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AmBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, BridgeError) as e:
        if isinstance(e, BridgeError):
            print("am_bridge: %s" % e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
