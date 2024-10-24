from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_supervisor_node = Node(
        package="vehicle_bringup",
        executable="vehicle_supervisor_node",
    )

    return LaunchDescription(
        [
            vehicle_supervisor_node
        ]
    )
