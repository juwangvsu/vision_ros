import sys, time, glob, subprocess, traceback, itertools, yaml, os, functools, pathlib, uuid
import xml.etree.ElementTree as ET
import rclpy

from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.actions import (
    LogInfo,
    EmitEvent,
    RegisterEventHandler,
)
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

import abc
import grpc
import json
import uuid
import time
import numpy as np
from vision_grpc.vision_grpc import message_pb2
from vision_grpc.vision_grpc import serve_pb2_grpc
from google.protobuf import any_pb2
from google.protobuf import text_format


def shutdown_on_process_exit(f):
    def g(*args):
        entries = f(*args)
        if entries:
            entries += [
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=entries[0],
                        on_exit=[
                            LogInfo(
                                msg=f"Node {f.__name__} exited; tearing down entire system."
                            ),
                            EmitEvent(event=Shutdown()),
                        ],
                    ),
                ),
            ]
        return entries

    return g


class RobotDescription:
    def moveit_config(self):
        with open(
            os.path.realpath(
                os.path.join(
                    get_package_share_directory("vision_ros"),
                    "launch",
                    "config",
                    "controllers.yaml",
                )
            )
        ) as stream:
            controller = yaml.safe_load(stream)
            xacro = ET.parse(
                os.path.realpath(
                    os.path.join(
                        get_package_share_directory("vision_ros"),
                        "launch",
                        "description",
                        "base.urdf.xacro",
                    )
                )
            ).getroot()
            entries = list(
                map(
                    lambda e: e.attrib,
                    filter(
                        lambda e: e.tag == "{http://www.ros.org/wiki/xacro}property",
                        xacro,
                    ),
                )
            )

            wheel_separation = list(
                filter(lambda e: e["name"] == "wheel_separation", entries)
            )
            assert len(wheel_separation), "no wheel_separation exists in xacro"
            assert all(
                float(entry["value"])
                == controller["diff_drive_controller"]["ros__parameters"][
                    "wheel_separation"
                ]
                for entry in wheel_separation
            ), f"discrepancies in xacro and controller.yaml for wheel_separation: {wheel_separation}"

            wheel_radius = list(filter(lambda e: e["name"] == "wheel_radius", entries))
            assert len(wheel_radius), "no wheel_radius exists in xacro"
            assert all(
                float(entry["value"])
                == controller["diff_drive_controller"]["ros__parameters"][
                    "wheel_radius"
                ]
                for entry in wheel_radius
            ), f"discrepancies in xacro and controller.yaml for wheel_radius: {wheel_radius}"

        return (
            MoveItConfigsBuilder("vision", package_name="vision_moveit_config")
            .robot_description(
                file_path="config/vision.urdf.xacro",
            )
            .robot_description_semantic(file_path="config/vision.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .to_moveit_configs()
        )


class StreamUnaryClientCallDetailsInterceptor(grpc.StreamUnaryClientInterceptor):
    def __init__(self, function):
        self.function = function

    def intercept_stream_unary(
        self, continuation, client_call_details, request_iterator
    ):
        return continuation(self.function(client_call_details), request_iterator)


class UnaryStreamClientCallDetailsInterceptor(grpc.UnaryStreamClientInterceptor):
    def __init__(self, function):
        self.function = function

    def intercept_unary_stream(self, continuation, client_call_details, request):
        return continuation(self.function(client_call_details), request)


class ClientCallDetailsCallable(abc.ABC):
    @abc.abstractmethod
    def __call__(self, client_call_details):
        raise NotImplementedError("__call__")


class ClientMetadata(ClientCallDetailsCallable):
    def __init__(self, metadata):
        self.metadata = metadata

    def __call__(self, client_call_details):
        metadata = list(
            [] if client_call_details.metadata is None else client_call_details.metadata
        ) + list(self.metadata)
        details = grpc.ClientCallDetails()
        details.method = client_call_details.method
        details.timeout = client_call_details.timeout
        details.metadata = metadata
        details.credentials = client_call_details.credentials
        details.wait_for_ready = client_call_details.wait_for_ready
        details.compression = client_call_details.compression
        return details


class LoggingClientCallDetails(ClientCallDetailsCallable):
    def __init__(self, logger):
        self.logger = logger

    def __call__(self, client_call_details):
        self.logger.info(f">{client_call_details.method} {client_call_details}")
        return client_call_details


class FunctionLogger:
    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    def info(self, msg, *args, **kwargs):
        self.logger.info(" I " + msg, *args, **kwargs)


class FunctionNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("grpc", rclpy.Parameter.Type.STRING)
        self.grpc = self.get_parameter("grpc").get_parameter_value().string_value

        self.declare_parameter("device", rclpy.Parameter.Type.STRING)
        self.device = str(
            uuid.UUID(self.get_parameter("device").get_parameter_value().string_value)
        )

        self.logger = FunctionLogger(self.get_logger())

        self.channel = None

        self.get_logger().info(f"grpc: {self.grpc} - device: {self.device}")

    def grpc_channel(self, interceptor_class):
        json_config = json.dumps(
            {
                "methodConfig": [
                    {
                        "name": [{"service": "Serve"}],
                        "retryPolicy": {
                            "maxAttempts": 5,
                            "initialBackoff": "1s",
                            "maxBackoff": "10s",
                            "backoffMultiplier": 2,
                            "retryableStatusCodes": ["UNAVAILABLE"],
                        },
                    }
                ]
            }
        )
        if self.grpc == "127.0.0.1:50051":
            channel = grpc.insecure_channel(
                self.grpc,
                options=[("grpc.service_config", json_config)],
            )
        else:
            channel = grpc.secure_channel(
                self.grpc,
                grpc.ssl_channel_credentials(),
                options=[("grpc.service_config", json_config)],
            )
        return grpc.intercept_channel(
            channel,
            interceptor_class(
                ClientMetadata(
                    [
                        ("device", self.device),
                    ]
                )
            ),
            interceptor_class(LoggingClientCallDetails(self.get_logger())),
        )
