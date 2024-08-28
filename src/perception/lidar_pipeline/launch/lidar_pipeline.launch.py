import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_pipeline",
                executable="lidar_detector_node",
                parameters=[
                    os.path.join(get_package_share_path("lidar_pipeline"), "config", "lidar_detector.yaml"),
                ],
            ),
            # Node(
            #     package="ground_plane_segmenter",
            #     executable="ground_plane_segmenter_node",
            #     parameters=[
            #         os.path.join(get_package_share_path("ground_plane_segmenter"), "config", "ground_plane_segmenter.yaml"),
            #     ],
            # ),
        ]
    )
