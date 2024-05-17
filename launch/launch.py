import sys, time, glob, subprocess, traceback, itertools, yaml, os, functools, pathlib, uuid
import xml.etree.ElementTree as ET
import rclpy

from launch import LaunchDescription
from launch.actions import (
    LogInfo,
    EmitEvent,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.actions import (
    Node,
    SetParameter,
    ComposableNodeContainer,
    LoadComposableNodes,
)
from launch_param_builder import ParameterBuilder
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from vision_ros import function


class DeviceInformation:
    def __init__(self, name):
        lookup = {
            "1546:01a8:u-blox_GNSS_receiver": "gps",
            "10c4:ea60:CP2102_USB_to_UART_Bridge": "lidar",  # RPLidar A2m12
        }

        def f(name):
            rclpy.logging.get_logger("launch").info(f"[{name}] 1enabled")
            lines = subprocess.run(
                ["udevadm", "info", name], capture_output=True
            ).stdout.splitlines()
            vendor = next(filter(lambda e: e.startswith(b"E: ID_VENDOR_ID="), lines))[
                16:
            ].decode()
            model = next(filter(lambda e: e.startswith(b"E: ID_MODEL_ID="), lines))[
                15:
            ].decode()
            product = next(filter(lambda e: e.startswith(b"E: ID_MODEL="), lines))[
                12:
            ].decode()
            rclpy.logging.get_logger("launch").info(f"[{name}] 2enabled")
            return f"{vendor}:{model}:{product}"

        def g(key, lookup):
            return next(
                filter(
                    lambda name: (lookup.get(f(name), None) == key),
                    sorted(
                        list(
                            itertools.chain(
                                glob.glob("/dev/input/js*"),
                                glob.glob("/dev/ttyACM*"),
                                glob.glob("/dev/ttyUSB*"),
                                glob.glob("/dev/robot/video*"),
                            )
                        )
                    ),
                )
            )

        self.device = g(name, lookup)

    def port(self):
        return self.device

    def rate(self):
        lines = subprocess.run(
            [f"stty < {self.device}"], capture_output=True, shell=True
        ).stdout.splitlines()
        entry = next(
            filter(
                lambda e: e.startswith(b"speed ") and e.endswith(b" baud; line = 0;"),
                lines,
            )
        )
        return int(
            entry.removeprefix(b"speed ").removesuffix(b" baud; line = 0;").decode()
        )


class ModuleDescription:
    def __init__(self, module):
        self.call = module

    def __call__(self):
        return getattr(self, self.call)() if hasattr(self, self.call) else list()

    def manage(self):
        return [
            "task",
            "post",
            "info",
            "throttle:odom",
            "throttle:navsat",
        ]

    def driver(self):
        return [
            "imu",
            "gps",
            "gpio",
            "joint",
            "lidar",
            "temperature",
        ]

    def vision(self):
        return [
            "slam_toolbox",
            "robot_localization",
            "robot_state_publisher",
            "ros2_controller",
            "vision",
        ]

    def upgrade(self):
        return [
            "upgrade",
        ]


class ThrottleDescription:
    def __init__(self, record):
        self.call = record

    def __call__(self):
        return getattr(self, self.call)() if hasattr(self, self.call) else list()

    @function.shutdown_on_process_exit
    def odom(self):
        return [
            Node(
                executable="throttle",
                package="topic_tools",
                namespace="/vision/throttle",
                output="screen",
                arguments=[
                    "messages",
                    "/odom",
                    "1.0",
                    "/vision/message/odom",
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                name="odom",
            ),
        ]

    @function.shutdown_on_process_exit
    def image(self):
        return [
            Node(
                executable="throttle",
                package="topic_tools",
                namespace="/vision/throttle",
                output="screen",
                arguments=[
                    "messages",
                    "/rgb",
                    "1.0",
                    "/vision/message/image",
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                name="image",
            ),
        ]

    @function.shutdown_on_process_exit
    def navsat(self):
        return [
            Node(
                executable="throttle",
                package="topic_tools",
                namespace="/vision/throttle",
                output="screen",
                arguments=[
                    "messages",
                    "/gps/filtered",
                    "1.0",
                    "/vision/message/navsat",
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                name="navsat",
            ),
        ]


class RecordDescription:
    def __init__(self, record, device=None):
        self.call = record
        self.device = device

    def __call__(self):
        return (
            getattr(self, self.call)()
            if hasattr(self, self.call)
            else (
                ThrottleDescription(self.call.removeprefix("throttle:"))()
                if self.call.startswith("throttle:")
                else list()
            )
        )

    def upgrade(self):
        return [
            Node(
                executable="upgrade",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                respawn=True,
            )
        ]

    @function.shutdown_on_process_exit
    def gps(self):
        device = DeviceInformation("gps")
        return [
            Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "port": device.port(),
                        "baudrate": device.rate(),
                        "frame_id": "gps",
                        "time_ref_source": "gps",
                        "useRMC": False,
                    }
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def post(self):
        return [
            Node(
                executable="post",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "grpc": "127.0.0.1:50051",
                        "device": self.device,
                    }
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def task(self):
        return [
            Node(
                executable="task",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "grpc": "127.0.0.1:50051",
                        "device": self.device,
                    }
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def info(self):
        return [
            Node(
                executable="info",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def imu(self):
        return [
            Node(
                package="bno055",
                executable="bno055",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    PathJoinSubstitution(
                        [
                            ThisLaunchFileDir(),
                            "config",
                            "bno055_params_i2c.yaml",
                        ]
                    ),
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def gpio(self):
        return [
            Node(
                namespace="/vision/function",
                package="vision_ros",
                executable="gpio",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def joint(self):
        return [
            Node(
                namespace="/vision/function",
                package="vision_ros",
                executable="joint",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "joint_commands_topic": "/diff_drive_controller/joint_commands",
                        "joint_states_topic": "/diff_drive_controller/joint_states",
                    }
                ],
                name="base_joint",
            ),
            Node(
                namespace="/vision/function",
                package="vision_ros",
                executable="joint",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "joint_commands_topic": "/joint_trajectory_controller/joint_commands",
                        "joint_states_topic": "/joint_trajectory_controller/joint_states",
                    }
                ],
                name="arm_joint",
            ),
        ]

    @function.shutdown_on_process_exit
    def marker(self):
        return [
            Node(
                executable="relay_field",
                package="topic_tools",
                namespace="/vision/marker",
                output="screen",
                arguments=[
                    "/servo_node/pose_target_cmds",
                    "/marker",
                    "visualization_msgs/Marker",
                    "{header: {frame_id: 'base_link'}, type: 2, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {r: 1.0, g: 1.0, b: 0.0, a: 1.0}, pose: {position: {x: m.pose.position.x, y: m.pose.position.y, z: m.pose.position.z}, orientation: {x: m.pose.orientation.x, y: m.pose.orientation.y, z: m.pose.orientation.x, w: m.pose.orientation.w}}}",
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                name="marker",
            ),
        ]

    @function.shutdown_on_process_exit
    def lidar(self):
        device = DeviceInformation("lidar")
        return [
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": device.port(),
                        "serial_baudrate": 256000,  # RPLidar A2m12
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
            )
        ]

    def temperature(self):
        return [
            Node(
                executable="temperature",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                respawn=True,
            )
        ]

    def slam_toolbox(self):
        return [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("slam_toolbox"),
                            "config",
                            "mapper_params_online_async.yaml",
                        ]
                    ),
                ],
                respawn=True,
            )
        ]

    @function.shutdown_on_process_exit
    def robot_localization(self):
        return [
            Node(
                package="robot_localization",
                executable="ekf_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    PathJoinSubstitution(
                        [
                            ThisLaunchFileDir(),
                            "config",
                            "ekf.yaml",
                        ]
                    ),
                ],
                remappings=[
                    ("odometry/filtered", "odom"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {"frequency": 1.0},
                    {"delay": 3.0},
                    {"magnetic_declination_radians": 0.0},
                    {"publish_filtered_gps": True},
                ],
                remappings=[
                    ("odometry/filtered", "odom"),
                    ("gps/fix", "fix"),
                    ("imu", "bno055/imu"),
                ],
            ),
        ]

    def robot_state_publisher(self):
        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    function.RobotDescription().moveit_config().robot_description,
                ],
                respawn=True,
            )
        ]

    @function.shutdown_on_process_exit
    def ros2_controller(self):
        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs",
            ],
            parameters=[
                function.RobotDescription().moveit_config().robot_description,
                PathJoinSubstitution(
                    [
                        ThisLaunchFileDir(),
                        "config",
                        "controllers.yaml",
                    ]
                ),
            ],
        )

        joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager-timeout",
                "300",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--disable-external-lib-logs",
            ],
        )

        diff_drive_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "diff_drive_controller",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--disable-external-lib-logs",
            ],
        )

        panda_arm_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "panda_arm_controller",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--disable-external-lib-logs",
            ],
        )

        panda_hand_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "panda_hand_controller",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--disable-external-lib-logs",
            ],
        )

        servo_params = {
            "moveit_servo": ParameterBuilder("vision_ros")
            .yaml(os.path.join("launch", "config", "servo.yaml"))
            .yaml(os.path.join("launch", "config", "pose_tracking_settings.yaml"))
            .to_dict()
        }

        moveit_servo = Node(
            package="moveit_servo",
            executable="servo_pose_tracking_demo",
            output="screen",
            parameters=[
                servo_params,
                function.RobotDescription().moveit_config().robot_description,
                function.RobotDescription().moveit_config().robot_description_semantic,
                function.RobotDescription()
                .moveit_config()
                .robot_description_kinematics,
            ],
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs",
            ],
            name="servo_node",
        )

        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[function.RobotDescription().moveit_config().to_dict()],
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs",
            ],
        )

        return [
            controller_manager,
            joint_state_broadcaster,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[
                        LogInfo(
                            msg=f"Node joint_state_broadcaster exited; start controllers."
                        ),
                        diff_drive_controller,
                        panda_arm_controller,
                        panda_hand_controller,
                    ],
                )
            ),
            moveit_servo,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=moveit_servo,
                    on_start=[
                        LogInfo(msg=f"Node moveit_servo started; wait 5 seconds."),
                        TimerAction(
                            period=5.0,
                            actions=[
                                LogInfo(
                                    msg=f"Node moveit_servo waited 5 seconds; start service."
                                ),
                                ExecuteProcess(
                                    cmd=[
                                        "/ros_entrypoint.sh",
                                        "ros2",
                                        "service",
                                        "call",
                                        "/servo_node/start_servo",
                                        "std_srvs/srv/Trigger",
                                        "{}",
                                    ],
                                ),
                            ],
                        ),
                    ],
                )
            ),
            move_group,
        ]

    @function.shutdown_on_process_exit
    def vision(self):
        return [
            Node(
                executable="vision",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "bbox": "/bbox",
                        "image": "/vision/message/image",
                        "image_bbox": "/image_bbox",
                        "image_masks": "/image_masks",
                        "image_depth": "/image_depth",
                    }
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def tracking(self):
        return [
            Node(
                executable="tracking",
                package="vision_ros",
                namespace="/vision/function",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
            )
        ]

    @function.shutdown_on_process_exit
    def image(self):
        return [
            Node(
                executable="image_publisher_node",
                package="image_publisher",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "filename": os.path.join(
                            get_package_share_directory("rviz_common"),
                            "images",
                            "splash.png",
                        )
                    }
                ],
            )
        ]


class HealthDescription:
    def __init__(self, health):
        self.param = health

    def __call__(self):
        return (
            self.health(self.param)
            if self.param
            in [
                "manage",
                "driver",
                "vision",
                "upgrade",
            ]
            else list()
        )

    @function.shutdown_on_process_exit
    def health(self, param):
        return [
            Node(
                executable="health",
                package="vision_ros",
                namespace="/vision/health",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "param": param,
                    }
                ],
                name=param,
            )
        ]


def generate_launch_description():
    descriptions = LaunchDescription()

    device = next(
        map(
            lambda e: str(uuid.UUID(e.removeprefix("device:=").zfill(32))),
            filter(lambda e: e.startswith("device:="), sys.argv),
        ),
        None,
    )
    device = (
        next(
            map(
                lambda e: str(uuid.UUID(e.strip().split(":")[1].strip().zfill(32))),
                filter(
                    lambda e: len(e.strip().split(":")) == 2
                    and e.strip().split(":")[0].strip() == "Serial",
                    pathlib.Path("/proc/cpuinfo").read_text().split("\n"),
                ),
            ),
            None,
        )
        if device is None
        else device
    )
    device = "12345678-1234-5678-1234-567812345678" if device is None else device
    device = str(uuid.UUID(device))
    rclpy.logging.get_logger("launch").info(f"[{device}] device")

    for record in (
        set(
            functools.reduce(
                lambda a, b: a + b,
                map(
                    lambda e: ModuleDescription(e.removeprefix("launch:="))(),
                    filter(lambda e: e.startswith("launch:="), sys.argv),
                ),
                [],
            )
        )
        | set(
            functools.reduce(
                lambda a, b: a + b,
                map(
                    lambda e: [e.removeprefix("on:=")],
                    filter(lambda e: e.startswith("on:="), sys.argv),
                ),
                [],
            )
        )
    ) - set(
        functools.reduce(
            lambda a, b: a + b,
            map(
                lambda e: [e.removeprefix("off:=")],
                filter(lambda e: e.startswith("off:="), sys.argv),
            ),
            [],
        )
    ):
        rclpy.logging.get_logger("launch").info(f"[{record}] enabled")
        for action in RecordDescription(record, device=device)():
            descriptions.add_action(action)

    if "off:=health" not in sys.argv:
        for health in set(
            functools.reduce(
                lambda a, b: a + b,
                map(
                    lambda e: [e.removeprefix("launch:=")],
                    filter(lambda e: e.startswith("launch:="), sys.argv),
                ),
                [],
            )
        ):
            rclpy.logging.get_logger("launch").info(f"[{health}] enabled")
            for action in HealthDescription(health)():
                descriptions.add_action(action)

    return descriptions
