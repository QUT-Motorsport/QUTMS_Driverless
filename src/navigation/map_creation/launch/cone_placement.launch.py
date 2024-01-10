import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="map_creation",
                executable="cone_placement_node",
                parameters=[
                    os.path.join(get_package_share_path("map_creation"), "config", "cone_placement.yaml"),
                ],
            )
        ]
    )
