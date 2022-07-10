from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_pipeline",
                executable="trt_detector",
            ),
            Node(
                package="lidar_pipeline",
                executable="lidar_processing",
            ),
            Node(
                package="sim_pursuit", # will rename package in refactor
                executable="local_pursuit",
            ),
            ## MAPPING NODE

            ## MPC NODE (PURE PURSUIT FOR NOW) - dormant until loop closure service call
        ]
    )
