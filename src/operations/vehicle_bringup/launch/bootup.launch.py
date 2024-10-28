import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_supervisor_node = Node(
        package="vehicle_bringup",
        executable="vehicle_supervisor_node",
    )

    mission_launcg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("nav_bringup"), "launch", "mission.launch.py")
        )
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_path("nav_bringup"), "launch", "sbg.launch.py"))
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            sbg_launch,
            vehicle_supervisor_node,
        ]
    )
