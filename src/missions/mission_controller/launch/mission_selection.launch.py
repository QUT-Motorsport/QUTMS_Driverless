# # need launch file to start mission selector
# # launch file calls other launch files for the specific mission

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"

    can_node = Node(
        package="mission_controller",
        executable="dummy_can",
    )

    mission_node = Node(
        package="mission_controller",
        executable="mission_control",
        env=node_env,
    )

    mission_pkg = get_package_share_directory("mission_controller")

    return LaunchDescription(
        [
            can_node,
            mission_node,
            RegisterEventHandler(
                OnProcessIO(
                    target_action=mission_node,
                    on_stdout=lambda event: IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([mission_pkg, "/", event.text.decode().strip(), ".launch.py"]),
                    ),
                )
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
