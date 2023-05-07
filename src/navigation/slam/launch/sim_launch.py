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
                    ("cone_detection", "vision/cone_detection"),
                ],
                parameters=[
                    {"range_variance": 0.5},
                    {"bearing_variance": 0.02},
                    {"uncertanty_time_weight": 0.005},
                    {"uncertanty_heading_time_weight": 0.005},
                    {"association_dist_threshold": 2.0},
                    {"use_total_abs_vel": False},
                    {"use_known_assocation": False},
                ],
            ),
        ]
    )
