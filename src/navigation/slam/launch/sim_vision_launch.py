from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# bearing 0.00872665

# # range 0.01
# RANGE_VAR = 0.05
# BEARING_VAR = 0.01
# ASSOCIATION = 1.0

# # range 0.3
# RANGE_VAR = 0.5
# BEARING_VAR = 0.01
# ASSOCIATION = 1.0

# range 0.3
RANGE_VAR = 0.5
BEARING_VAR = 0.01
ASSOCIATION = 1.0


def generate_launch_description():
    range_variance = LaunchConfiguration("range_variance")
    use_known_association = LaunchConfiguration("use_known_association")

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="range_variance"),
            DeclareLaunchArgument(name="use_known_association"),
            Node(
                package="slam",
                executable="node_ekf_slam",
                name="ekf_slam",
                remappings=[
                    ("velocity", "imu/velocity"),
                    ("cone_detection", "vision/cone_detection"),
                ],
                parameters=[
                    {"range_variance": range_variance},
                    {"bearing_variance": 0.01},
                    {"uncertanty_time_weight": 0.005},
                    {"uncertanty_heading_time_weight": 0.00005},
                    {"association_dist_threshold": 1.0},
                    {"use_total_abs_vel": False},
                    {"use_known_association": use_known_association},
                    {"use_odom_only": False},
                    {"reverse_rotation": False},
                ],
            ),
        ]
    )
