from setuptools import find_packages, setup

import glob, os

package_name = "vision_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            glob.glob(os.path.join("resource", package_name)),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "launch", "config"),
            glob.glob(os.path.join("launch", "config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "launch", "description"),
            glob.glob(os.path.join("launch", "description", "*.xacro")),
        ),
        (
            os.path.join("share", package_name, "launch", "description"),
            glob.glob(os.path.join("launch", "description", "*.urdf")),
        ),
        (
            os.path.join("share", package_name, "launch", "description", "meshes"),
            glob.glob(os.path.join("launch", "description", "meshes", "*.stl")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task = vision_ros.task:main",
            "post = vision_ros.post:main",
            "info = vision_ros.info:main",
            "daemon = vision_ros.daemon:main",
            "health = vision_ros.health:main",
            "upgrade = vision_ros.upgrade:main",
            "tracking = vision_ros.tracking:main",
            "temperature = vision_ros.temperature:main",
            "battery = vision_ros.battery:main",
            "camera = vision_ros.camera:main",
            "vision = vision_ros.vision:main",
            "joint = vision_ros.joint:main",
            "gpio = vision_ros.gpio:main",
        ],
    },
)
