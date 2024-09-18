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
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[("cloud_in", "/lidar/cone_points"), ("scan", "/lidar/converted_2D_scan")],
                parameters=[
                    {
                        "target_frame": "velodyne",
                        "transform_tolerance": 0.01,
                        "min_height": -1.0,
                        "max_height": -0.5,
                        "angle_min": -1.5708,
                        "angle_max": 1.5708,
                        "angle_increment": 0.004363,  # M_PI/720 degrees
                        "scan_time": 0.1,
                        "range_min": 0.5,
                        "range_max": 25.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                name="pointcloud_to_laserscan_node",
            ),
        ]
    )
