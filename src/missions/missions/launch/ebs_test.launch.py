from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="controllers",
                executable="reactive_control",
                parameters=[{"ebs_control": True}],
            ),
            Node(
                package="vision_pipeline",
                executable="torch_detector",
            ),
            Node(
                package="lidar_pipeline_2",
                executable="lidar_perception",
            ),
            Node(
                package="baby_slam",
                executable="slam",
            ),
        ]
    )
