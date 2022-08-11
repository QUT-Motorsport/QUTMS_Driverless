import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_path = os.path.join(get_package_share_path("driverless_common"), "display.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="rviz", default_value="false"),
            DeclareLaunchArgument(
                name="rvizconfig", default_value=str(rviz_path), description="Absolute path to rviz config file"
            ),
            Node(
                package="driverless_common",
                executable="display",
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
