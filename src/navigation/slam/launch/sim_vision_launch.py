from launch import LaunchDescription
from launch_ros.actions import Node

# bearing 0.00872665

# range 0.01
RANGE_VAR = 0.05
BEARING_VAR = 0.01


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
                    {"range_variance": RANGE_VAR},
                    {"bearing_variance": BEARING_VAR},
                    # {"uncertanty_time_weight": 0.005},
                    # {"uncertanty_heading_time_weight": 0.8},
                    {"uncertanty_time_weight": 0.005},
                    {"uncertanty_heading_time_weight": 0.00005},
                    {"association_dist_threshold": 2.0},
                    {"use_total_abs_vel": False},
                    {"use_known_assocation": False},
                    {"use_odom_only": False},
                    {"reverse_rotation": False},
                ],
            ),
        ]
    )
