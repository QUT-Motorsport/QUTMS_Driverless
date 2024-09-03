import os

from ament_index_python.packages import get_package_share_path
import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    # Copied from: https://github.com/ros-drivers/velodyne/blob/galactic-devel/velodyne/launch/velodyne-all-nodes-VLP32C-launch.py
    driver_params_file = str(get_package_share_path("sensors") / "config" / "VLP32C.yaml")

    transform_params_file = os.path.join(get_package_share_path("sensors") / "config" / "VLP32C.yaml")
    with open(transform_params_file, "r") as f:
        transform_params = yaml.safe_load(f)["velodyne_transform_node"]["ros__parameters"]
    transform_params["calibration"] = str(get_package_share_path("sensors") / "config" / "VLP32C-calibration.yaml")

    laserscan_params_file = str(get_package_share_path("sensors") / "config" / "VLP32C.yaml")

    gps_params_file = str(get_package_share_path("ground_plane_segmenter") / "config" / "ground_plane_segmenter.yaml")

    container = ComposableNodeContainer(
        name="velodyne_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="velodyne_driver",
                plugin="velodyne_driver::VelodyneDriver",
                name="velodyne_driver_node",
                parameters=[driver_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="velodyne_pointcloud",
                plugin="velodyne_pointcloud::Transform",
                name="velodyne_transform_node",
                parameters=[transform_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="velodyne_laserscan",
                plugin="velodyne_laserscan::VelodyneLaserScan",
                name="velodyne_laserscan_node",
                parameters=[laserscan_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="ground_plane_segmenter",
                plugin="ground_plane_segmenter::GroundPlaneSegmenterNode",
                name="ground_plane_segmenter_node",
                parameters=[gps_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    return launch.LaunchDescription([container])
