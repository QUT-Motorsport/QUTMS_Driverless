from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="controllers",  # will rename package in refactor
                executable="reactive_control",
            ),
            ## MAPPING NODE
        ]
    )
