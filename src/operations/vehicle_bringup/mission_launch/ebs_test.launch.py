import datetime
import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    record = LaunchConfiguration("record")

    # nav2 stack
    nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("nav_bringup"), "launch", "nav_stack_bringup.launch.py")
        )
    )
    # mapping/planning
    cone_matcher_node = Node(
        package="map_creation",
        executable="cone_placement_node",
        parameters=[
            get_package_share_path("map_creation") / "config" / "cone_placement.yaml",
        ],
        output="both",
    )
    grid_to_cone_node = Node(
        package="slam_gridmap",
        executable="gridmap_to_cone_detection_node",
        output="both",
    )
    planner_node = Node(
        package="planners",
        executable="ft_planner_node",
        parameters=[
            get_package_share_path("planners") / "config" / "ft_planner.yaml",
        ],
        output="both",
    )
    # control translation
    ackermann_control_node = Node(
        package="controllers",
        executable="vel_to_ackermann_node",
        parameters=[{"Kp": 2.0}],  # specific for EBS test
        output="both",
    )

    # recording
    # get current time for bag file name
    now = datetime.datetime.now()
    name = f'bags/ebs_test-{now.strftime("%Y-%m-%d-%H-%M-%S")}'
    ros2_bag_node = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap", "-o", name, "--all", "-x", "(/velodyne_points|/velodyne_packets)"],
        output="both",
        condition=IfCondition(record),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_record_cmd = DeclareLaunchArgument(
        "record",
        default_value="true",
        description="Record the bag file",
    )

    return LaunchDescription(
        [
            declare_record_cmd,
            stdout_linebuf_envvar,
            nav_stack_launch,
            cone_matcher_node,
            grid_to_cone_node,
            planner_node,
            ackermann_control_node,
            ros2_bag_node,
        ]
    )
