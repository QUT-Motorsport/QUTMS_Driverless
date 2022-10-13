import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    can_node = Node(
        package="mission_controller",
        executable="dummy_can",
    )

    mission_node = Node(
        package="mission_controller",
        executable="mission_control",
    )

    model_pkg = get_package_share_directory("models")
    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(model_pkg + "/launch/robot_description.launch.py")
    )

    return LaunchDescription(
        [
            robot_model,
            can_node,
            mission_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
