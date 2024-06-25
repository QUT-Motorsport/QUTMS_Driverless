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
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/lidar_debug/cone_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'velodyne',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            # 'angle_min': -math.pi/2,
            # 'angle_max': math.pi/2,
            # 'angle_increment': math.pi/720,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 25.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    sbg_translator_node = Node(
        package="sbg_translator",
        executable="sbg_translator_node",
    ),

    lidar_detector_node = Node(
        package="lidar_pipeline",
        executable="lidar_detector_node",
    ),

    return launch.LaunchDescription(
        [
            localisation_node,
            sbg_translator_node,
            async_slam_toolbox_node,
            pointcloud_to_laserscan_node,
            lidar_detector_node,
        ]
    )
