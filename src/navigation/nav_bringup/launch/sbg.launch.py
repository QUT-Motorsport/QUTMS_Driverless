import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav_bringup")
    sbg_config = os.path.join(bringup_dir, "config", "EllipseD.yaml")

    sbg_driver_node = Node(
        package="sbg_driver",
        executable="sbg_device",
        name="sbg_driver_node",
        output="both",
        parameters=[sbg_config],
    )

    return LaunchDescription(
        [
            sbg_driver_node,
        ]
    )
