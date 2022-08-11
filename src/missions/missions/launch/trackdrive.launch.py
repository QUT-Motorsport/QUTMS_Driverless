from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    testing = DeclareLaunchArgument(name="testing", default_value="False")

    return LaunchDescription(
        [
            testing,
            Node(
                package="missions",
                executable="trackdrive",
                parameters=[{"testing": LaunchConfiguration("testing")}],
            ),
            Node(
                package="controllers",
                executable="reactive_control",
            ),
            # Node(
            #     package="baby_slam",
            #     executable="slam",
            # )
            # Node(
            #     package="vision_pipeline",
            #     executable="torch_detector",
            # ),
            # Node(
            #     package="lidar_pipeline_2",
            #     executable="lidar_processing",
            # ),
            # Node(
            #     package="fusion",
            #     executable="fusion",
            # ),
        ]
    )
