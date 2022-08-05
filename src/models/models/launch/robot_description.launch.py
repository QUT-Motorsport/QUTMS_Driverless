import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_model = LaunchConfiguration("urdf_model")
    rviz_path = os.path.join(get_package_share_path("models"), "rviz", "display.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="rviz", default_value="false"),
            DeclareLaunchArgument(
                name="rvizconfig", default_value=str(rviz_path), description="Absolute path to rviz config file"
            ),
            DeclareLaunchArgument(
                "urdf_model", default_value="qev3.urdf.xacro", description="URDF Model to use (from models/urdf)"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            ["xacro ", f"{get_package_share_path('models') / 'urdf'}/", urdf_model]
                        )
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(  # maybe have an arg here too?
                package="driverless_common",
                executable="display",
            ),
            Node(
                package="transforms",
                executable="transform",
            ),
        ]
    )
