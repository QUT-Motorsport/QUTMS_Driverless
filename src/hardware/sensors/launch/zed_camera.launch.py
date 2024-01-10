import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # URDF/xacro file to be loaded by the Robot State Publisher node
    camera_config_path = os.path.join(get_package_share_directory("sensors"), "config", "zed_camera.yaml")

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package="zed_wrapper",
        namespace="zed2i",
        executable="zed_wrapper",
        name="zed_node",
        output="screen",
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            camera_config_path,  # Camera related parameters
        ],
    )

    return LaunchDescription([zed_wrapper_node])
