from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="controllers",
                executable="reactive_control",
                parameters=[{"ebs_control": True}],
            ),
            # Node(
            #     package="controllers",
            #     executable="vector_reactive_control",
            #     parameters=[{"ebs_control": True}],
            # ),
            # Node(
            #     package="controllers",
            #     executable="simple_straight_control",
            # ),
            # Node(
            #     package="controllers",
            #     executable="constant",
            # ),
        ]
    )
