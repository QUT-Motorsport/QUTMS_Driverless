from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="missions",
                executable="inspection",
            ),
            Node(
                package="controllers",
                executable="sine",
            ),
        ]
    )
