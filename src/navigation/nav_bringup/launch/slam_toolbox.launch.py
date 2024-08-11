import os

from ament_index_python.packages import get_package_share_path
import launch
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
        remappings=[
            ("cmd_vel", "control/nav_cmd_vel"),
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

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config/pointcloud_to_laserscan_params.yaml'),
        ],
        remappings=[('cloud_in', '/lidar/cone_points'),
                    ('scan', '/lidar/converted_2D_scan')],
    ),

    return launch.LaunchDescription(
        [
            localisation_node,
            async_slam_toolbox_node,
            pointcloud_to_laserscan_node,
        ]
    )
