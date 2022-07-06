import os

import ament_index_python.packages
import launch
import launch_ros.actions
import yaml


def generate_launch_description():
    return launch.LaunchDescription(velodyne_launch())


def velodyne_launch():
    # Copied from: https://github.com/ros-drivers/velodyne/blob/galactic-devel/velodyne/launch/velodyne-all-nodes-VLP32C-launch.py
    driver_share_dir = ament_index_python.packages.get_package_share_directory("velodyne_driver")
    driver_params_file = os.path.join(driver_share_dir, "config", "VLP32C-velodyne_driver_node-params.yaml")
    velodyne_driver_node = launch_ros.actions.Node(
        package="velodyne_driver", executable="velodyne_driver_node", output="both", parameters=[driver_params_file]
    )

    convert_share_dir = ament_index_python.packages.get_package_share_directory("velodyne_pointcloud")
    convert_params_file = os.path.join(convert_share_dir, "config", "VLP32C-velodyne_convert_node-params.yaml")
    with open(convert_params_file, "r") as f:
        convert_params = yaml.safe_load(f)["velodyne_convert_node"]["ros__parameters"]
    convert_params["calibration"] = os.path.join(convert_share_dir, "params", "VeloView-VLP-32C.yaml")
    velodyne_convert_node = launch_ros.actions.Node(
        package="velodyne_pointcloud", executable="velodyne_convert_node", output="both", parameters=[convert_params]
    )

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory("velodyne_laserscan")
    laserscan_params_file = os.path.join(laserscan_share_dir, "config", "default-velodyne_laserscan_node-params.yaml")
    velodyne_laserscan_node = launch_ros.actions.Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        output="both",
        parameters=[laserscan_params_file],
    )

    return [
        velodyne_driver_node,
        velodyne_convert_node,
        velodyne_laserscan_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]


def sbg_launch():
    # Copied from: https://github.com/SBG-Systems/sbg_ros2_driver/blob/master/launch/sbg_device_launch.py

    config = os.path.join(
        ament_index_python.packages.get_package_share_directory("sensors"), "config", "sbg_config.yaml"
    )

    return [launch_ros.actions.Node(package="sbg_driver", executable="sbg_device", output="both", parameters=[config])]
