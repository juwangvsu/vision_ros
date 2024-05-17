import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
import simple_pid
import numpy as np

# ros2 topic pub /servo_node/pose_target_cmds geometry_msgs/PoseStamped '{pose: {position: {x: 0.5675, y: 0.1555, z: 0.6000}, orientation: {x: 0.9490, y: 0.0008, z: 0.0003, w: 0.3153}}}'
class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking")

        self.pid = None

        self.subscription = self.create_subscription(
            PoseStamped,
            "/servo_node/pose_target_cmds",
            self.subscription_callback,
            10,
        )

        self.publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.timer = self.create_timer(0.1, self.callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def callback(self):
        if self.pid is not None:
            try:
                t = self.tf_buffer.lookup_transform(
                    "panda_link0", "panda_hand", rclpy.time.Time()
                )
            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform panda_hand to base_link: {ex}"
                )
                return
            self.get_logger().info(
                f"current position  {t.transform.translation} {t.transform.rotation}"
            )

            calculated = np.clip(
                self.pid(
                    np.array(
                        list(
                            [
                                t.transform.translation.x,
                                t.transform.translation.y,
                                t.transform.translation.z,
                            ]
                        )
                        + list(
                            euler_from_quaternion(
                                [
                                    t.transform.rotation.x,
                                    t.transform.rotation.y,
                                    t.transform.rotation.z,
                                    t.transform.rotation.w,
                                ]
                            )
                        ),
                        dtype=np.float64,
                    )
                ),
                -1.0,
                1.0,
            )
            self.get_logger().info(f"PID: => {calculated} => {self.pid.setpoint}")

            if np.linalg.norm(calculated) < 0.02:
                self.pid = None
                return

            x, y, z, roll, pitch, yaw = calculated

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "panda_link0"
            msg.twist.linear.x = x
            msg.twist.linear.y = y
            msg.twist.linear.z = z
            msg.twist.angular.x = roll
            msg.twist.angular.y = pitch
            msg.twist.angular.z = yaw

            self.publisher.publish(msg)

    def subscription_callback(self, msg):
        if self.pid is None:
            self.get_logger().info(f"Received /servo_node/pose_target_cmds {msg}")
            try:
                t = self.tf_buffer.lookup_transform(
                    "panda_hand", "base_link", rclpy.time.Time()
                )
            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform panda_hand to base_link: {ex}"
                )
                return
            self.pid = simple_pid.PID(
                setpoint=np.array(
                    list(
                        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                    )
                    + list(
                        euler_from_quaternion(
                            [
                                msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w,
                            ]
                        )
                    ),
                    dtype=np.float64,
                )
            )


def main(args=None):
    rclpy.init(args=args)

    node = TrackingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
