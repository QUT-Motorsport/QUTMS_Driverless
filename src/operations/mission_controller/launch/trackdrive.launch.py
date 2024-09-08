import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # odom
            Node(
                package="odom_transformer",
                executable="odom_transformer_node",
            ),
            # localisation
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path("nav_bringup"), "launch", "slam_toolbox.launch.py")
                )
            ),
            # mapping/planning
            Node(
                package="map_creation",
                executable="cone_placement_node",
                parameters=[
                    get_package_share_path("map_creation") / "config" / "cone_placement.yaml",
                ],
            ),
            Node(
                package="planners",
                executable="ft_planner_node",
                parameters=[
                    get_package_share_path("planners") / "config" / "ft_planner.yaml",
                ],
                ros_arguments=[
                    "-p", "topic_name:=slam/global_map",
                    "-p", "target_frame:=track",
                ]
            ),
            # guidance/control
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path("nav_bringup"), "launch", "nav2_bringup.launch.py")
                )
            ),
            Node(
                package="controllers",
                executable="vel_to_ackermann_node",
                parameters=[{"Kp": 4.0}], # specific for Trackdrive
            ),
        ]
    )
