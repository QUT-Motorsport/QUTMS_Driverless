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

    velodyne_pointcloud_container = ComposableNodeContainer(
        name="velodyne_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="velodyne_driver",
                plugin="velodyne_driver::VelodyneDriver",
                name="velodyne_driver_node",
                parameters=[vlp32_params_file],
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
                parameters=[vlp32_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=[],
    )

    urdf_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py")
        ),
        launch_arguments=[
            ("urdf_model", "qev-3d.urdf.xacro"),
            ("base_frame", "base_footprint"),
            ("display_car", "false"),
        ],
    )

    return LaunchDescription(
        [
            velodyne_pointcloud_container,
            rviz_node,
            urdf_launch,
        ]
    )
