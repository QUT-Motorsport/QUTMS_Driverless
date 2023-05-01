from ament_index_python.packages import get_package_share_path
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
            Node(
                package="steering_actuator",
                executable="steering_actuator_node",
                parameters=[
                    get_package_share_path("steering_actuator") / "config" / "steering.yaml",
                ],
            ),
        ]
    )
