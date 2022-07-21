from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_pipeline",
                executable="trt_detector",
            ),
            Node(
                package="lidar_pipeline",
                executable="lidar_processing",
            ),
            Node(
                package="controllers",  # will rename package in refactor
                executable="reactive_control",
            ),
        ]
    )
