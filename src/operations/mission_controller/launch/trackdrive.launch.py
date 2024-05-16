import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sbg_translator",
                executable="sbg_translator_node",
            ),
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
            ),
            # Node(
            #     package="pure_pursuit_cpp",
            #     executable="pure_pursuit_node",
            #     parameters=[
            #         get_package_share_path("pure_pursuit_cpp") / "config" / "pure_pursuit_cpp.yaml",
            #     ],
            # ),
            Node(
                package="controllers",
                executable="vel_to_ackermann_node",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path("nav_bringup"), "launch", "slam_toolbox.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path("nav_bringup"), "launch", "nav2_bringup.launch.py")
                )
            ),
        ]
    )
