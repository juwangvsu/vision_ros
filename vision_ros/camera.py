import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import subprocess


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")

        self.declare_parameter("port", "")

        self.port = str(self.get_parameter("port").value)
        self.get_logger().info(f'Port: "{self.port}"')
        assert self.port

        self.publisher_ = self.create_publisher(String, "camera", 10)

    def callback(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = CameraNode()

    with subprocess.Popen(
        [
            "ffmpeg",
            "-f",
            "v4l2",
            "-i",
            node.port,
            "-f",
            "v4l2",
            "-codec:v",
            "rawvideo",
            "-pix_fmt",
            "yuv420p",
            "/dev/video0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    ) as process:
        for line in process.stdout:
            node.callback(line.decode().strip())

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
