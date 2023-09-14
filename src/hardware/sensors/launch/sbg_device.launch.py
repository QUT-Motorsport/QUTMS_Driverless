import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("sensors"), "config", "ellipse_D.yaml")

    return LaunchDescription(
        [
            Node(
                package="sbg_driver",
                executable="sbg_device",
                output="screen",
                parameters=[config],
            ),
        ]
    )
