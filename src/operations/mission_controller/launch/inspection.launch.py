from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="inspection_mission",
            ),
            Node(
                package="controllers",
                executable="sine",
            ),
        ]
    )
