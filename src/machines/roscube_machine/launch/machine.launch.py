from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="canbus",
                executable="canbus",
                parameters=[
                    "/home/developer/driverless_ws/src/hardware/canbus/config/canbus.yaml"
                ],  # TODO: use auto getter for path
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    [get_package_share_directory("sensors"), "/launch/sensors.launch.py"]
                ),
            ),
        ]
    )
