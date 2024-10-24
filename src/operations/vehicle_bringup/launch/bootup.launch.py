import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_supervisor_node = Node(
        package="vehicle_bringup",
        executable="vehicle_supervisor_node",
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_path("nav_bringup"), "launch", "sbg.launch.py"))
    )

    return LaunchDescription([sbg_launch, vehicle_supervisor_node])
