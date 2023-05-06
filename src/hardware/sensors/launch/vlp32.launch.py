import os

from ament_index_python.packages import get_package_share_path
import launch
import launch_ros.actions
import yaml


def generate_launch_description():
    # Copied from: https://github.com/ros-drivers/velodyne/blob/galactic-devel/velodyne/launch/velodyne-all-nodes-VLP32C-launch.py
    driver_params_file = str(
        get_package_share_path("velodyne_driver") / "config" / "VLP32C-velodyne_driver_node-params.yaml"
    )
    velodyne_driver_node = launch_ros.actions.Node(
        package="velodyne_driver", executable="velodyne_driver_node", output="both", parameters=[driver_params_file]
    )

    convert_params_file = os.path.join(
        get_package_share_path("velodyne_pointcloud") / "config" / "VLP32C-velodyne_convert_node-params.yaml"
    )
    with open(convert_params_file, "r") as f:
        convert_params = yaml.safe_load(f)["velodyne_convert_node"]["ros__parameters"]
    convert_params["calibration"] = str(
        get_package_share_path("velodyne_pointcloud") / "params" / "VeloView-VLP-32C.yaml"
    )
    velodyne_convert_node = launch_ros.actions.Node(
        package="velodyne_pointcloud", executable="velodyne_convert_node", output="both", parameters=[convert_params]
    )

    laserscan_params_file = str(
        get_package_share_path("velodyne_laserscan") / "config" / "default-velodyne_laserscan_node-params.yaml"
    )
    velodyne_laserscan_node = launch_ros.actions.Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        output="both",
        parameters=[laserscan_params_file],
    )

    return launch.LaunchDescription(
        [
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
    )
