#!/usr/bin/env python3
"""Launch the hospital world, explicit bridges, and one mission controller."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    package_share = Path(get_package_share_directory("smartcar_bringup"))
    project_root = package_share.parents[3]
    world = project_root / "worlds" / "hospital_world_gz10.world"
    bridge_config = package_share / "config" / "ros_gz_bridge.yaml"
    route_config = package_share / "config" / "mission_routes.json"
    headless = LaunchConfiguration("headless").perform(context).lower() in {"1", "true", "yes"}

    gz_command = ["gz", "sim", "-r", "--render-engine", "ogre2"]
    if headless:
        gz_command.append("-s")
    gz_command.append(str(world))
    resource_path = os.pathsep.join(
        [
            str(project_root / "models"),
            "/opt/ros/lyrical/share/turtlebot3_gazebo/models",
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    ).rstrip(os.pathsep)

    return [
        ExecuteProcess(
            cmd=gz_command,
            output="screen",
            additional_env={"GZ_SIM_RESOURCE_PATH": resource_path, "DISPLAY": os.environ.get("DISPLAY", ":99")},
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="hospital_ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": str(bridge_config)}],
        ),
        Node(
            package="smartcar_bringup",
            executable="mission_controller",
            name="hospital_mission_controller",
            output="screen",
            parameters=[{"route_file": str(route_config), "use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
