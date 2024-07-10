import os

from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_path("nav_bringup")

    # Community ROS 2 packages
    localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/localisation_params.yaml"),
        ],
    )

    async_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/slam_toolbox_params.yaml"),
        ],
        remappings=[
            ("/pose", "/slam/car_pose"),
            ("/slam_toolbox/graph_visualization", "/slam/graph_visualisation"),
        ],
    )

    point_cloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/lidar/cone_points'),
                    ('scan', '/lidar/converted_2D_scan')],
        parameters=[{
            'target_frame': 'velodyne',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'angle_increment': 0.004363,  # M_PI/720 degrees
            'scan_time': 0.1,
            'range_min': 0.5,
            'range_max': 25.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan_node'
    )

    return launch.LaunchDescription(
        [
            localisation_node,
            async_slam_toolbox_node,
            point_cloud_to_laserscan_node
        ]
    )
