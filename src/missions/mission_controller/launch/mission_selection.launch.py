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
    # ros2 print by default is cleared from std out, hence would not be picked up as an std i/o.
    # to overcome this, the host terminal environment is saved and python is set to an 'unbuffered' std out
    # which allows ros2 print statements to be registered in std out.
    # this is then piped into the 'mission_control' node.

    can_node = Node(
        package="mission_controller",
        executable="dummy_can",
    )

    mission_node = Node(
        package="mission_controller",
        executable="mission_control",
        env=node_env,
    )

    model_pkg = get_package_share_directory("models")
    robot_model = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                model_pkg + '/launch/robot_description.launch.py'))

    mission_pkg = get_package_share_directory("missions")

    return LaunchDescription(
        [
            robot_model,
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
