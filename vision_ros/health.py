import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import datetime


class HealthNode(Node):
    def __init__(self):
        super().__init__("health")

        self.declare_parameter("param", rclpy.Parameter.Type.STRING)
        self.param = self.get_parameter("param").get_parameter_value().string_value

        self.get_logger().info(f"Health: {self.param}")

        if hasattr(self, f"health_{self.param}"):
            getattr(self, f"health_{self.param}")()
        else:
            self.health_none()
        self.publisher = self.create_publisher(String, f"{self.param}", 10)

    def health_none(self):
        self.timer = self.create_timer(1.0, self.health_none_timer_callback)

    def health_none_timer_callback(self):
        msg = String()
        msg.data = f"Health: {self.param}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def health_upgrade(self):
        self.health_upgrade_subscription = self.create_subscription(
            String,
            "/vision/function/upgrade",
            self.health_upgrade_subscription_callback,
            10,
        )
        self.health_upgrade_timestamp = self.get_clock().now()
        self.get_logger().info(
            "I start: {}".format(
                datetime.datetime.utcfromtimestamp(
                    self.health_upgrade_timestamp.seconds_nanoseconds()[0]
                )
            )
        )

        self.timer = self.create_timer(1.0, self.health_upgrade_timer_callback)

    def health_upgrade_subscription_callback(self, msg):
        self.health_upgrade_timestamp = self.get_clock().now()
        self.get_logger().info(
            "I heard: {} {}".format(
                datetime.datetime.utcfromtimestamp(
                    self.health_upgrade_timestamp.seconds_nanoseconds()[0]
                ),
                msg,
            )
        )

    def health_upgrade_timer_callback(self):
        timestamp = self.get_clock().now()
        if timestamp - self.health_upgrade_timestamp > rclpy.duration.Duration(
            seconds=1800
        ):
            self.get_logger().info(
                'Publishing: Timeout - "{}" - "{}"'.format(
                    datetime.datetime.utcfromtimestamp(
                        self.health_upgrade_timestamp.seconds_nanoseconds()[0]
                    ),
                    datetime.datetime.utcfromtimestamp(
                        timestamp.seconds_nanoseconds()[0]
                    ),
                )
            )
            return
        msg = String()
        msg.data = f"Health: {self.param}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: Upgrade - "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = HealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
