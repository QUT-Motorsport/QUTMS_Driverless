import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="trackdrive_handler_node",
            ),
            Node(
                package="controllers",
                executable="reactive_control_node",
            ),
            Node(
                package="pure_pursuit",
                executable="pure_pursuit_node",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_package_share, "launch", "cone_association_slam.launch.py")
                )
            ),
        ]
    )
