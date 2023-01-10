import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    driver_share_dir = get_package_share_directory("velodyne_driver")
    driver_params_file = os.path.join(driver_share_dir, "config", "VLP32C-velodyne_driver_node-params.yaml")
    convert_share_dir = get_package_share_directory("velodyne_pointcloud")
    convert_params_file = os.path.join(convert_share_dir, "config", "VLP32C-velodyne_transform_node-params.yaml")
    with open(convert_params_file, "r") as f:
        convert_params = yaml.safe_load(f)["velodyne_transform_node"]["ros__parameters"]
    convert_params["calibration"] = os.path.join(convert_share_dir, "params", "VeloView-VLP-32C.yaml")
    laserscan_share_dir = get_package_share_directory("velodyne_laserscan")
    laserscan_params_file = os.path.join(laserscan_share_dir, "config", "default-velodyne_laserscan_node-params.yaml")

    return LaunchDescription(
        [
            Node(
                package="canbus",
                executable="canbus",
                parameters=[
                    get_package_share_path("canbus") / "config" / "canbus.yaml",
                ],
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            Node(
                package="steering_actuator",
                executable="steering",
                parameters=[
                    get_package_share_path("steering_actuator") / "config" / "steering.yaml",
                ],
            ),
            Node(
                package="vehicle_supervisor",
                executable="vehicle_supervisor",
            ),
            Node(
                package="car_status",
                executable="car_status_node",
            ),
            Node(
                package="driverless_common",
                executable="display",
            ),
            Node(
                package="lidar_pipeline_3",
                executable="lidar_perception",
            ),
            Node(
                package="mission_controller",
                executable="mission_control",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(get_package_share_path("sbg_driver") / "launch" / "sbg_device_launch.py")
                ),
            ),
            # temporary velodyne replacement while galactic conversion is broken
            # IncludeLaunchDescription(
            #     launch_description_source=PythonLaunchDescriptionSource(
            #         launch_file_path=str(
            #             get_package_share_path("velodyne") / "launch" / "velodyne-all-nodes-VLP32C-launch.py"
            #         )
            #     ),
            # ),
            Node(
                package="velodyne_driver",
                executable="velodyne_driver_node",
                output="both",
                parameters=[driver_params_file],
            ),
            Node(
                package="velodyne_pointcloud",
                executable="velodyne_transform_node",
                output="both",
                parameters=[convert_params],
            ),
            Node(
                package="velodyne_laserscan",
                executable="velodyne_laserscan_node",
                output="both",
                parameters=[laserscan_params_file],
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py"
                    )
                ),
                launch_arguments=[
                    ("urdf_model", "qev3.urdf.xacro"),
                ],
            ),
        ]
    )
