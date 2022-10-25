from launch import LaunchDescription

from rclpy.node import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="driverless_common",
                executable="display",
            ),
        ]
    )
