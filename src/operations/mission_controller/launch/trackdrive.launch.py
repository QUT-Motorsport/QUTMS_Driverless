from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="controllers",
            #     executable="reactive_control",
            # ),
            Node(
                package="steering_actuator",
                executable="steering",
                parameters=[
                    get_package_share_path("steering_actuator") / "config" / "steering.yaml",
                ],
            ),
            Node(
                package="velocity_controller",
                executable="velocity_controller",
                parameters=[
                    get_package_share_path("velocity_controller") / "config" / "velocity_controller.yaml",
                ],
            ),
        ]
    )
