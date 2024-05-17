import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

import ament_index_python
import ctypes
import time
import os


class JointNode(Node):
    def __init__(self):
        super().__init__("joint")

        self.declare_parameter("joint_commands_topic", rclpy.Parameter.Type.STRING)
        self.joint_commands_topic = (
            self.get_parameter("joint_commands_topic")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"Joint: {self.joint_commands_topic}")

        self.declare_parameter("joint_states_topic", rclpy.Parameter.Type.STRING)
        self.joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Joint: {self.joint_states_topic}")

        self.publisher = self.create_publisher(
            JointState,
            self.joint_states_topic,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.subscription = self.create_subscription(
            JointState,
            self.joint_commands_topic,
            self.subscription_callback,
            10,
        )

    def subscription_callback(self, msg):
        self.get_logger().info(
            f'I heard: "{list(msg.name)}: {list(msg.position)} {list(msg.velocity)} {list(msg.effort)}"'
        )
        states = msg

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = states.name
        msg.position = [
            states.position[i] if i < len(states.position) else 0.0
            for i, _ in enumerate(states.name)
        ]
        msg.velocity = [
            states.velocity[i] if i < len(states.velocity) else 0.0
            for i, _ in enumerate(states.name)
        ]
        msg.effort = [
            states.effort[i] if i < len(states.effort) else 0.0
            for i, _ in enumerate(states.name)
        ]
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Publishing: "{list(msg.name)}: {list(msg.position)} {list(msg.velocity)} {list(msg.effort)}"'
        )


def main(args=None):
    rclpy.init(args=args)

    node = JointNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
