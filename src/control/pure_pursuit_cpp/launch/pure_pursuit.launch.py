from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pure_pursuit_cpp",
                executable="node_pure_pursuit",
                parameters=[
                    get_package_share_path("pure_pursuit_cpp") / "config" / "pure_pursuit_cpp.yaml",
                ],
            ),
            Node(
            package="pure_pursuit_cpp",
            executable="lifecycle_pure_pursuit",
            parameters=[
                get_package_share_path("pure_pursuit_cpp") / "config" / "pure_pursuit_cpp.yaml",
                ],
            )
        ]
    )
