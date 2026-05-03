import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_supervisor_node = Node(
        package="vehicle_bringup",
        executable="vehicle_supervisor_node",
        condition=UnlessCondition(EnvironmentVariable("PUSHCART")),
    )

    system_watcher = Node(
        package="vehicle_bringup",
        executable="system_watcher_node",
        condition=UnlessCondition(EnvironmentVariable("PUSHCART")),
    )

    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("vehicle_bringup"), "launch", "system.launch.py")
        )
    )

    launch_system_on_watcher_exit = GroupAction(
        actions=[
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=system_watcher,
                    on_exit=[system_launch],
                )
            )
        ],
        condition=UnlessCondition(EnvironmentVariable("PUSHCART")),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("vehicle_bringup"), "launch", "mission.launch.py")
        ),
        condition=UnlessCondition(EnvironmentVariable("PUSHCART")),
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_path("nav_bringup"), "launch", "sbg.launch.py"))
    )

    rosbag = Node(
        package="rosbag2_transport",
        executable="recorder",
        name="rosbag_recorder",
        parameters=os.path.join(get_package_share_path("vehicle_bringup"), "config", "rosbag.yaml"),
    )

    pushcart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("vehicle_bringup"), "launch", "push_log.launch.py")
        ),
        condition=IfCondition(EnvironmentVariable("PUSHCART")),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0")

    return LaunchDescription(
        [
            system_launch,
            stdout_linebuf_envvar,
            sbg_launch,
            rosbag,
            pushcart_launch,
            system_watcher,
            launch_system_on_watcher_exit,
            mission_launch,
            vehicle_supervisor_node,
        ]
    )
