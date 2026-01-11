import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    # Velodyne VLP32C params
    # Copied from: https://github.com/ros-drivers/velodyne/blob/galactic-devel/velodyne/launch/velodyne-all-nodes-VLP32C-launch.py
    vlp32_params_file = str(get_package_share_path("perception_bringup") / "config" / "VLP32C.yaml")

    transform_params_file = os.path.join(get_package_share_path("perception_bringup") / "config" / "VLP32C.yaml")
    with open(transform_params_file, "r") as f:
        transform_params = yaml.safe_load(f)["velodyne_transform_node"]["ros__parameters"]
    transform_params["calibration"] = str(
        get_package_share_path("perception_bringup") / "config" / "VLP32C-calibration.yaml"
    )

    gps_params_file = str(get_package_share_path("ground_plane_segmenter") / "config" / "ground_plane_segmenter.yaml")

    pointcloud_to_laserscan_params_file = os.path.join(
        get_package_share_path("perception_bringup"), "config", "pointcloud_to_laserscan.yaml"
    )

    pointcloud_container = ComposableNodeContainer(
        name="pointcloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="velodyne_driver",
            #     plugin="velodyne_driver::VelodyneDriver",
            #     name="velodyne_driver_node",
            #     parameters=[vlp32_params_file],
            #     extra_arguments=[{"use_intra_process_comms": True}],
            # ),
            # ComposableNode(
            #     package="velodyne_pointcloud",
            #     plugin="velodyne_pointcloud::Transform",
            #     name="velodyne_transform_node",
            #     parameters=[transform_params],
            #     extra_arguments=[{"use_intra_process_comms": True}],
            # ),
            # ComposableNode(
            #     package="velodyne_laserscan",
            #     plugin="velodyne_laserscan::VelodyneLaserScan",
            #     name="velodyne_laserscan_node",
            #     parameters=[vlp32_params_file],
            #     extra_arguments=[{"use_intra_process_comms": True}],
            # ),
            # composed node for faster msg transfer
            ComposableNode(
                package="points_maker",
                plugin="points_maker::PointsMakerNode",
                name="points_maker_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="ground_plane_segmenter",
                plugin="ground_plane_segmenter::GroundPlaneSegmenterNode",
                name="ground_plane_segmenter_node",
                parameters=[gps_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="pointcloud_to_laserscan_node",
                parameters=[pointcloud_to_laserscan_params_file],
                remappings=[("cloud_in", "/lidar/objects"), ("scan", "/lidar/converted_2D_scan")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="both",
    )

    lidar_detection_params_file = os.path.join(
        get_package_share_path("lidar_pipeline"), "config", "lidar_detector.yaml"
    )

    lidar_detector_node = Node(
        package="lidar_pipeline",
        executable="lidar_detector_node",
        output="screen",
        parameters=[lidar_detection_params_file],
    )


    return LaunchDescription(
        [
            velodyne_pointcloud_container,
            lidar_detector_node,
        ]
    )
