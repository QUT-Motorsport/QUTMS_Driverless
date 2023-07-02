import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_package_share = get_package_share_path("qutms_nav2")
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="trackdrive_handler",
            ),
            Node(
                package="controllers",
                executable="reactive_control",
            ),
            Node(
                package="path_follower",
                executable="pure_pursuit",
            ),
            Node(
                package="planners",
                executable="ordered_mid_spline",
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(nav_package_share, "launch", "cone_association_slam.launch.py")
            #     ),
            #     launch_arguments={
            #         "slam_params_file": os.path.join(nav_package_share, "config/slam_params.yaml")
            #     }.items(),
            # )
            # Node(
            #     package="steering_actuator",
            #     executable="steering_actuator_node",
            #     parameters=[
            #         get_package_share_path("steering_actuator") / "config" / "steering.yaml",
            #     ],
            # ),
            # Node(
            #     package="velocity_controller",
            #     executable="velocity_controller_node",
            #     parameters=[
            #         get_package_share_path("velocity_controller") / "config" / "velocity_controller.yaml",
            #     ],
            # ),
        ]
    )
