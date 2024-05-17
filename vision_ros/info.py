import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import abc
import grpc
import json
import uuid
import time
import netifaces
import itertools
import numpy as np
from urllib.request import urlopen
from vision_grpc.vision_grpc import message_pb2
from vision_grpc.vision_grpc import serve_pb2_grpc
from google.protobuf import any_pb2
from google.protobuf import text_format
from vision_ros import daemon


class InfoNode(Node):
    def __init__(self):
        super().__init__("information")

        self.publisher = self.create_publisher(String, "/vision/message/info", 10)
        self.subscription = self.create_subscription(
            String,
            "/vision/task/info",
            self.subscription_callback,
            10,
        )
        self.timer = self.create_timer(10.0, self.callback)

    def subscription_callback(self, msg):
        self.get_logger().info(f' I heard: "{msg}"')
        self.callback()

    def callback(self):
        try:
            container, endpoints, distribution, configuration = daemon.conf()
            address = ",".join(
                sorted(
                    filter(
                        lambda e: e not in endpoints.values(),
                        map(
                            lambda e: e["addr"],
                            itertools.chain.from_iterable(
                                map(
                                    lambda e: netifaces.ifaddresses(e)[
                                        netifaces.AF_INET
                                    ],
                                    filter(
                                        lambda e: e != "lo" and e != "docker0",
                                        netifaces.interfaces(),
                                    ),
                                )
                            ),
                        ),
                    )
                )
            )
            certificate = json.loads(
                urlopen("http://localhost:2019/pki/ca/local").read().decode().strip()
            )["root_certificate"]
            information = {
                "mode": configuration["mode"],
                "daemon": ",".join(sorted(list(set(endpoints.values())))),
                "channel": configuration["pull"],
                "address": address,
                "certificate": certificate,
            }

            msg = String()
            msg.data = json.dumps(information)
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
        except Exception as e:
            self.get_logger().info(f'Exception: "{e}"')


def main(args=None):
    rclpy.init(args=args)

    node = InfoNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
