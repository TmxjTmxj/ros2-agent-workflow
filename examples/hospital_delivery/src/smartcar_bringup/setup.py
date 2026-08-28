import os
from glob import glob

from setuptools import find_packages, setup

package_name = "smartcar_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "config"), glob("../../config/mission_routes.json")),
        (os.path.join("share", package_name, "config"), glob("../../config/ros_gz_bridge.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agent ROS2 maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS2智能车国赛仿真作业系统",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_controller = smartcar_bringup.mission_controller:main",
        ],
    },
)
