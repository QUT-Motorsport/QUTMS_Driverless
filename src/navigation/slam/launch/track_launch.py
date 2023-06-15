from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="slam",
                executable="node_ekf_slam",
                name="ekf_slam",
                remappings=[
                    ("velocity", "imu/velocity"),
                    ("cone_detection", "lidar/cone_detection"),
                ],
                parameters=[
                    {"range_variance": 1.5},
                    {"bearing_variance": 0.5},
                    {"association_dist_threshold": 1.5},
                    {"use_total_abs_vel": True},
                ],
            ),
        ]
    )
