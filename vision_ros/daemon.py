import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os
import json
import yaml
import time
import shutil
import base64
import pathlib
import tempfile
import tenacity
import ipaddress
import itertools
import subprocess


class DaemonNode(Node):
    def __init__(self):
        super().__init__("daemon")

        self.declare_parameter("health", True)

    @tenacity.retry(stop=tenacity.stop_after_attempt(10), wait=tenacity.wait_fixed(15))
    def alive(self, endpoint, services):
        rclpy.logging.get_logger("daemon").info(f"Daemon[{endpoint}]: alive")
        containers = list(
            map(
                lambda service: json.loads(
                    subprocess.check_output(
                        [
                            "docker",
                            "--host",
                            endpoint,
                            "inspect",
                            f"robot-{service}",
                        ],
                        stderr=subprocess.STDOUT,
                        timeout=5,
                    ).decode()
                )[0],
                services,
            )
        )
        rclpy.logging.get_logger("daemon").info(
            f"Daemon[{endpoint}]: {list(map(lambda e: (e['Name'], e['State']), containers))}"
        )
        assert all(
            map(
                lambda e: e["State"]["Health"]["Status"] == "healthy",
                containers,
            )
        ), f"alive failure: {list(map(lambda e: (e['Name'], e['State']), containers))}"

    @tenacity.retry(stop=tenacity.stop_after_attempt(10), wait=tenacity.wait_fixed(30))
    def check(self, launches):
        rclpy.logging.get_logger("daemon").info(f"Daemon[{launches}]: check")
        statuses = list(
            map(
                lambda launch: (
                    launch,
                    subprocess.check_output(
                        [
                            "docker",
                            "exec",
                            "robot-manage",
                            "/ros_entrypoint.sh",
                            "ros2",
                            "topic",
                            "echo",
                            "--once",
                            f"/vision/health/{launch}",
                        ],
                        stderr=subprocess.STDOUT,
                        timeout=15,
                    ).decode(),
                ),
                launches,
            )
        )
        rclpy.logging.get_logger("daemon").info(f"Daemon[{launches}]: {statuses}")
        assert all(
            [
                (status == f"data: 'Health: {launch}'\n---\n")
                for launch, status in statuses
            ]
        ), f"check failure: {statuses}"


def conf():
    container = json.loads(
        subprocess.check_output(["docker", "inspect", "robot-daemon"], timeout=5)
    )[0]
    rclpy.logging.get_logger("daemon").info(
        f"Image: {container['Image']} {container['Config']['Image']}"
    )
    entries = next(
        filter(
            None,
            map(
                lambda e: e.removeprefix("daemon="),
                filter(
                    lambda e: e.startswith("daemon="),
                    json.loads(
                        subprocess.check_output(
                            ["docker", "info", "--format", "json"], timeout=5
                        )
                    )["Labels"],
                ),
            ),
        ),
        None,
    )
    endpoints = (
        dict(zip(["manage", "driver", "vision"], entries.split(",")))
        if entries
        else dict(
            zip(["manage", "driver", "vision"], ["unix:///var/run/docker.sock"] * 3)
        )
    )
    distribution = "{}:5000/vision".format(
        endpoints["manage"] if entries else "127.0.0.1"
    )
    rclpy.logging.get_logger("daemon").info(
        f"Endpoints: {endpoints} Distributin: {distribution}"
    )
    with open("/opt/ros/vision/config/conf.yaml") as f:
        configuration = yaml.safe_load(f)
        try:
            with open("/data/conf.yaml") as g:
                configuration.update(yaml.safe_load(g))
        except:
            pass
    rclpy.logging.get_logger("daemon").info(f"Configuration: {configuration}")
    return container, endpoints, distribution, configuration


def sync(endpoint, image, distribution):
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: ls: {image}")
    images = list(
        set(
            subprocess.check_output(
                [
                    "docker",
                    "--host",
                    endpoint,
                    "image",
                    "ls",
                    "--quiet",
                    "--no-trunc",
                ],
                stderr=subprocess.STDOUT,
                timeout=5,
            )
            .decode()
            .split()
        )
    )
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: ls {images}")
    if image in images:
        return

    status = subprocess.check_output(
        [
            "docker",
            "tag",
            image,
            distribution,
        ],
        stderr=subprocess.STDOUT,
        timeout=5,
    ).decode()
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: tag: {status}")

    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: push: {image}")
    status = subprocess.check_output(
        [
            "docker",
            "push",
            "--quiet",
            distribution,
        ],
        stderr=subprocess.STDOUT,
    ).decode()
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: push: {status}")

    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: inspect: {image}")
    digest = next(
        filter(
            lambda e: e.startswith(f"{distribution}@"),
            json.loads(
                subprocess.check_output(
                    [
                        "docker",
                        "image",
                        "inspect",
                        image,
                    ],
                    stderr=subprocess.STDOUT,
                    timeout=5,
                ).decode()
            )[0]["RepoDigests"],
        ),
        None,
    )
    assert digest, "Image {image} no digest"
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: inspect: {digest}")

    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: pull: {image}")
    status = subprocess.check_output(
        [
            "docker",
            "--host",
            endpoint,
            "pull",
            digest,
        ],
        stderr=subprocess.STDOUT,
    ).decode()
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: pull: {status}")

    images = list(
        set(
            subprocess.check_output(
                [
                    "docker",
                    "--host",
                    endpoint,
                    "image",
                    "ls",
                    "--quiet",
                    "--no-trunc",
                ],
                stderr=subprocess.STDOUT,
                timeout=5,
            )
            .decode()
            .split()
        )
    )
    rclpy.logging.get_logger("daemon").info(f"Sync[{endpoint}]: {images}")

    assert image in images, "Image {image} not found"


def main(args=None):
    rclpy.init(args=args)

    node = DaemonNode()
    try:
        container, endpoints, distribution, configuration = conf()

        with open("/opt/ros/vision/config/node.yaml") as f:
            services = yaml.safe_load(f)["services"]

        mode = configuration["mode"]

        override = {
            "services": {
                name: {"image": container["Image"]}
                for name, service in services.items()
            }
        }
        for entry, endpoint in endpoints.items():
            sync(endpoint, container["Image"], distribution)

            profile = f"{entry}:{mode}"
            config = subprocess.check_output(
                [
                    "docker",
                    "--host",
                    endpoint,
                    "compose",
                    "--project-name",
                    "robot",
                    "--profile",
                    profile,
                    "--file",
                    "/opt/ros/vision/config/node.yaml",
                    "--file",
                    "/dev/stdin",
                    "config",
                ],
                input=yaml.dump(override).encode(),
                stderr=subprocess.STDOUT,
                timeout=15,
            ).decode()
            rclpy.logging.get_logger("daemon").info(f"Load[{endpoint}]: {config}")

            status = subprocess.check_output(
                [
                    "docker",
                    "--host",
                    endpoint,
                    "compose",
                    "--project-name",
                    "robot",
                    "--profile",
                    profile,
                    "--file",
                    "/opt/ros/vision/config/node.yaml",
                    "--file",
                    "/dev/stdin",
                    "up",
                    "--detach",
                    "--force-recreate",
                    "--pull",
                    "never",
                    "--timeout",
                    "5",
                    "--wait",
                    "--wait-timeout",
                    "60",
                ],
                input=yaml.dump(override).encode(),
                stderr=subprocess.STDOUT,
                timeout=120,
            ).decode()
            rclpy.logging.get_logger("daemon").info(f"Load[{endpoint}]: {status}")

            rclpy.logging.get_logger("daemon").info(f"Load[{endpoints}]: prune")
            status = subprocess.check_output(
                [
                    "docker",
                    "--host",
                    endpoint,
                    "system",
                    "prune",
                    "--force",
                ],
                stderr=subprocess.STDOUT,
                timeout=60,
            ).decode()
            rclpy.logging.get_logger("daemon").info(
                f"Upgrade[{endpoints}]: prune {status}"
            )

        entries = [
            (
                endpoint,
                list(
                    filter(
                        lambda name: (
                            f"{endpoint}:{mode}" in services[name]["profiles"]
                        ),
                        services,
                    )
                ),
            )
            for endpoint in endpoints
        ]
        launches = list(
            itertools.chain.from_iterable(
                [
                    list(
                        map(
                            lambda e: e.removeprefix("launch:="),
                            filter(
                                lambda e: e.startswith("launch:="),
                                service["entrypoint"],
                            ),
                        )
                    )
                    for _, service in services.items()
                    if (
                        service["entrypoint"][:5]
                        == [
                            "/entrypoint.sh",
                            "ros2",
                            "launch",
                            "vision_ros",
                            "launch.py",
                        ]
                    )
                ]
            )
        )

        while True:
            rclpy.logging.get_logger("daemon").info(f"Alive[{entries}]")
            for entry, services in entries:
                node.alive(endpoints[entry], services)
            rclpy.logging.get_logger("daemon").info(f"Check[{launches}]")
            if node.get_parameter("health").get_parameter_value().bool_value:
                node.check(launches)
            time.sleep(10)
    except subprocess.CalledProcessError as e:
        rclpy.logging.get_logger("daemon").info(f"Daemon: error: {e.output.decode()}")
        raise e
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
