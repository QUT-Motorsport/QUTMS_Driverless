from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sim_pursuit",  # will rename package in refactor
                executable="local_pursuit",
            ),
            ## MAPPING NODE
        ]
    )
