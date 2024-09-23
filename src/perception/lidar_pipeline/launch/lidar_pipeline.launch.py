import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_path("lidar_pipeline")

    lidar_detector_node = Node(
        package="lidar_pipeline",
        executable="lidar_detector_node",
        parameters=[
            os.path.join(pkg_share, "config", "lidar_detector.yaml"),
        ],
    ),

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, "config", "pointcloud_to_laserscan_params.yaml"),
        ],
        remappings=[('cloud_in', '/lidar/cone_points'),
                    ('scan', '/lidar/converted_2D_scan')],
    ),

    return LaunchDescription(
        [
            lidar_detector_node,
            pointcloud_to_laserscan_node,
        ]
    )
