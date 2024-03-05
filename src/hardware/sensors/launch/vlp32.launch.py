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
    velodyne_driver_node = launch_ros.actions.Node(
        package="velodyne_driver", executable="velodyne_driver_node", output="both", parameters=[driver_params_file]
    )

    transform_params_file = os.path.join(get_package_share_path("sensors") / "config" / "VLP32C.yaml")
    with open(transform_params_file, "r") as f:
        transform_params = yaml.safe_load(f)["velodyne_transform_node"]["ros__parameters"]
    transform_params["calibration"] = str(get_package_share_path("sensors") / "config" / "VLP32C-calibration.yaml")
    velodyne_transform_node = launch_ros.actions.Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        output="both",
        parameters=[transform_params],
    )

    laserscan_params_file = str(get_package_share_path("sensors") / "config" / "VLP32C.yaml")
    velodyne_laserscan_node = launch_ros.actions.Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        output="both",
        parameters=[laserscan_params_file],
    )

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
                plugin="velodyne_pointcloud::Convert",  # Transform',
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
        ],
        output="both",
    )

    return launch.LaunchDescription([container])
