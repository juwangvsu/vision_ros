import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os
import json
import time
import shutil
import datetime
import subprocess
from vision_ros import daemon


class UpgradeNode(Node):
    def __init__(self):
        super().__init__("upgrade")

        self.publisher = self.create_publisher(
            String,
            f"upgrade",
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"Upgrade: OK"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        try:
            container, endpoints, distribution, configuration = daemon.conf()
            if "pull" not in configuration or "auth" not in configuration:
                rclpy.logging.get_logger("daemon").info(
                    f"Upgrade[{endpoints}]: no pull/auth"
                )
                return

            registry = configuration["pull"].split("/")[0]
            config = json.dumps({"auths": {registry: {"auth": configuration["auth"]}}})
            filename = "/root/.docker/config.json"
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            with open(filename, "w") as f:
                f.write(config)
            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: pull: {configuration['pull']}"
            )
            status = subprocess.check_output(
                [
                    "docker",
                    "pull",
                    configuration["pull"],
                ],
                stderr=subprocess.STDOUT,
            ).decode()
            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: pull: {status}"
            )
            shutil.rmtree(os.path.dirname(filename))

            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: inspect: {configuration['pull']}"
            )
            image = json.loads(
                subprocess.check_output(
                    [
                        "docker",
                        "image",
                        "inspect",
                        configuration["pull"],
                    ],
                    stderr=subprocess.STDOUT,
                    timeout=5,
                ).decode()
            )[0]["Id"]
            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: inspect: {image}"
            )

            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: tag: {image} {distribution}"
            )

            status = subprocess.check_output(
                [
                    "docker",
                    "tag",
                    image,
                    distribution,
                ],
                stderr=subprocess.STDOUT,
            ).decode()
            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: tag: {status} "
            )

            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: push {distribution}"
            )
            status = subprocess.check_output(
                [
                    "docker",
                    "push",
                    distribution,
                ],
                stderr=subprocess.STDOUT,
            ).decode()
            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: push {distribution}"
            )
            for entry, endpoint in endpoints.items():
                daemon.sync(endpoint, image, distribution)

            msg.data = f"Upgrade: Success"
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
        except Exception as e:
            msg.data = f"Upgrade: {e}"
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = UpgradeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
