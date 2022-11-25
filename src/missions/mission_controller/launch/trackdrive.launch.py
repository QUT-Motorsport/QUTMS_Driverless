from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_pipeline",
                executable="torch_detector",
            ),
            Node(
                package="lidar_pipeline_3",
                executable="lidar_perception",
            ),
            Node(
                package="py_slam",
                executable="slam",
            ),
            Node(
                package="planners",
                executable="delaunay_planner",
            ),
            Node(
                package="controllers",
                executable="pure_pursuit",
            ),
        ]
    )
