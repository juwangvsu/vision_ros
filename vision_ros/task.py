import rclpy
from rclpy.node import Node

from std_msgs.msg import String

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


class TaskNode(vision_ros.function.FunctionNode):
    def __init__(self):
        super().__init__("task")

        self.publisher = self.create_publisher(String, "task", 10)

    def grpc_task(self):
        try:
            request = any_pb2.Any()
            request.Pack(empty_pb2.Empty())
            self.logger.info(f">/Serve/RecvTask {request}")
            if self.channel is None:
                self.channel = self.grpc_channel(
                    vision_ros.function.UnaryStreamClientCallDetailsInterceptor
                )
            stub = serve_pb2_grpc.ServeStub(self.channel)
            for response in stub.RecvTask(
                request,
                timeout=5.0,
                metadata=[("function", "task")],
            ):
                self.logger.info(f"</Serve/RecvTask {response}")
        except grpc.RpcError as status:
            self.logger.info(f"{status}")
            self.channel.close() if self.channel is not None else None
            self.channel = None


def main(args=None):
    rclpy.init(args=args)

    node = TaskNode()

    try:
        while rclpy.ok():
            node.grpc_task()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
