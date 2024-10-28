import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    inspection_watcher = Node(
        package="vehicle_bringup",
        executable="inspection_watcher_node",
    )

    ebs_watcher = Node(
        package="vehicle_bringup",
        executable="ebs_watcher_node",
    )

    trackdrive_watcher = Node(
        package="vehicle_bringup",
        executable="trackdrive_watcher_node",
    )

    inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("nav_bringup"), "mission_launch", "inspection.launch.py")
        )
    )

    ebs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("nav_bringup"), "mission_launch", "ebs.launch.py")
        )
    )

    trackdrive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("nav_bringup"), "mission_launch", "trackdrive.launch.py")
        )
    )

    launch_inspection_on_watcher_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=inspection_watcher,
            on_exit=[inspection_launch],
        )
    )

    launch_ebs_on_watcher_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ebs_watcher,
            on_exit=[ebs_launch],
        )
    )

    launch_trackdrive_on_watcher_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=trackdrive_watcher,
            on_exit=[trackdrive_launch],
        )
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            inspection_watcher,
            ebs_watcher,
            trackdrive_watcher,
            launch_inspection_on_watcher_exit,
            launch_ebs_on_watcher_exit,
            launch_trackdrive_on_watcher_exit,
        ]
    )
