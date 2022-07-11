from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="canbus",
                executable="canbus",
                parameters=[
                    get_package_share_directory("canbus") / "config" / "canbus.yaml",
                ],
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    [get_package_share_directory("sensors"), "/launch/sensors.launch.py"]
                ),
            ),
        ]
    )
