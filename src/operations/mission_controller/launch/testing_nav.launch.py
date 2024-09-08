from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
            ),
            Node(
                package="driverless_common",
                executable="display",
            ),
            # Node(
            #     package="mission_controller",
            #     executable="mission_launcher_node",
            # ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py"
                    )
                ),
                launch_arguments=[
                    ("urdf_model", "qev-3d.urdf.xacro"),
                    ("base_frame", "base_footprint"),
                    ("display_car", "false"),
                ],
            ),
        ]
    )
