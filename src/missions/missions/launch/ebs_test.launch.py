from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="missions",
                executable="ebs_test",
            ),
            # Node(
            #     package="vision_pipeline",
            #     executable="trt_detector",
            # ),
            # Node(
            #     package="lidar_pipeline",
            #     executable="lidar_processing",
            # ),
            Node(
                package="controllers",
                executable="reactive_control",
            ),
        ]
    )
