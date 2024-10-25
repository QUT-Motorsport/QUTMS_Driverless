import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # nav2 stack
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path("nav_bringup"), "launch", "nav_stack_bringup.launch.py")
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
                package="slam_gridmap",
                executable="gridmap_to_cone_detection_node",
            ),
            Node(
                package="planners",
                executable="ft_planner_node",
                parameters=[
                    get_package_share_path("planners") / "config" / "ft_planner.yaml",
                ],
            ),
            # control translation
            Node(
                package="controllers",
                executable="vel_to_ackermann_node",
                parameters=[{"Kp": 4.0}],  # specific for Trackdrive
            ),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        ]
    )
