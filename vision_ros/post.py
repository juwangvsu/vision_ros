import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import abc
import grpc
import json
import uuid
import time
import numpy as np
from vision_grpc.vision_grpc import message_pb2
from vision_grpc.vision_grpc import serve_pb2_grpc
from google.protobuf import any_pb2
from google.protobuf import empty_pb2
from google.protobuf import text_format
import vision_ros.function


class deduplicate:
    def __init__(self, equal, interval):
        self.equal = equal
        self.interval = interval
        self.value = None
        self.returned = None
        self.perf_counter = None

    def __call__(self, function):
        def f(o, value):
            perf_counter = time.perf_counter()
            if (
                self.perf_counter is None
                or self.perf_counter + self.interval < perf_counter
                or not self.equal(self.value, value)
            ):
                self.value, self.returned, self.perf_counter = (
                    value,
                    function(o, value),
                    perf_counter,
                )
            return self.returned

        return f


def info_equal(a, b):
    return np.all(
        np.array(
            [
                a.mode,
                a.daemon,
                a.channel,
                a.address,
                a.certificate,
            ],
            dtype=np.string_,
        )
        == np.array(
            [
                b.mode,
                b.daemon,
                b.channel,
                b.address,
                b.certificate,
            ],
            dtype=np.string_,
        )
    )


def odom_equal(a, b):
    return (
        np.linalg.norm(
            np.array(
                [
                    a.pose.position.x,
                    a.pose.position.y,
                    a.pose.position.z,
                    a.pose.orientation.x,
                    a.pose.orientation.y,
                    a.pose.orientation.z,
                    a.pose.orientation.w,
                    a.twist.linear.x,
                    a.twist.linear.y,
                    a.twist.linear.z,
                    a.twist.angular.x,
                    a.twist.angular.y,
                    a.twist.angular.z,
                ],
                dtype=np.float64,
            )
            - np.array(
                [
                    b.pose.position.x,
                    b.pose.position.y,
                    b.pose.position.z,
                    b.pose.orientation.x,
                    b.pose.orientation.y,
                    b.pose.orientation.z,
                    b.pose.orientation.w,
                    b.twist.linear.x,
                    b.twist.linear.y,
                    b.twist.linear.z,
                    b.twist.angular.x,
                    b.twist.angular.y,
                    b.twist.angular.z,
                ],
                dtype=np.float64,
            )
        )
        < 1e-5
    )


def navsat_equal(a, b):
    return (
        np.linalg.norm(
            np.array(
                [
                    a.latitude,
                    a.longitude,
                    a.altitude,
                ],
                dtype=np.float64,
            )
            - np.array(
                [
                    a.latitude,
                    a.longitude,
                    a.altitude,
                ],
                dtype=np.float64,
            )
        )
        < 0.02
    )


class PostNode(vision_ros.function.FunctionNode):
    def __init__(self):
        super().__init__("post")

        self.info_subscription = self.create_subscription(
            String,
            "/vision/message/info",
            self.info_subscription_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/vision/message/odom",
            self.odom_subscription_callback,
            10,
        )
        self.navsat_subscription = self.create_subscription(
            NavSatFix,
            "/vision/message/navsat",
            self.navsat_subscription_callback,
            10,
        )

    def grpc_post(self, function, message):
        try:
            post = any_pb2.Any()
            post.Pack(message)
            self.logger.info(f">/Serve/SendPost {post}")
            if self.channel is None:
                self.channel = self.grpc_channel(
                    vision_ros.function.StreamUnaryClientCallDetailsInterceptor
                )
            stub = serve_pb2_grpc.ServeStub(self.channel)
            response = stub.SendPost(
                iter([post]),
                metadata=[
                    ("function", function),
                ],
            )
            self.logger.info(f"</Serve/SendPost {response}")
        except Exception as e:
            self.logger.info(f"</Serve/SendPost {e}")
            self.channel.close() if self.channel is not None else None
            self.channel = None

    def info_subscription_callback(self, msg):
        self.get_logger().info(f' I heard: "{msg}"')
        information = json.loads(msg.data)
        message = message_pb2.Information()
        message.mode = information["mode"]
        message.daemon = information["daemon"]
        message.channel = information["channel"]
        message.address = information["address"]
        message.certificate = information["certificate"]
        self.info_post(message)

    @deduplicate(info_equal, 60)
    def info_post(self, message):
        self.grpc_post("info", message)
        self.get_logger().info(
            f' I send: "{text_format.MessageToString(message, as_one_line=True)}"'
        )

    def odom_subscription_callback(self, msg):
        self.get_logger().info(f' I heard: "{msg}"')
        message = message_pb2.Odometry()
        message.pose.position.x = msg.pose.pose.position.x
        message.pose.position.y = msg.pose.pose.position.y
        message.pose.position.z = msg.pose.pose.position.z
        message.pose.orientation.x = msg.pose.pose.orientation.x
        message.pose.orientation.y = msg.pose.pose.orientation.y
        message.pose.orientation.z = msg.pose.pose.orientation.z
        message.pose.orientation.w = msg.pose.pose.orientation.w
        message.twist.linear.x = msg.twist.twist.linear.x
        message.twist.linear.y = msg.twist.twist.linear.y
        message.twist.linear.z = msg.twist.twist.linear.z
        message.twist.angular.x = msg.twist.twist.angular.x
        message.twist.angular.y = msg.twist.twist.angular.y
        message.twist.angular.z = msg.twist.twist.angular.z
        message.frame = msg.header.frame_id
        self.odom_post(message)

    @deduplicate(odom_equal, 60)
    def odom_post(self, message):
        self.grpc_post("odom", message)
        self.get_logger().info(
            f' I send: "{text_format.MessageToString(message, as_one_line=True)}"'
        )

    def navsat_subscription_callback(self, msg):
        self.get_logger().info(f' I heard: "{msg}"')
        message = message_pb2.NavSat()
        message.latitude = msg.latitude
        message.longitude = msg.longitude
        message.altitude = msg.altitude
        message.frame = msg.header.frame_id
        self.navsat_post(message)

    @deduplicate(navsat_equal, 300)
    def navsat_post(self, message):
        self.grpc_post("navsat", message)
        self.get_logger().info(
            f' I send: "{text_format.MessageToString(message, as_one_line=True)}"'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PostNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
