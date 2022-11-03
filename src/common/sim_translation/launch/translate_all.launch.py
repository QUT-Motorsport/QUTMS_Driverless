from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sim_translation",
                executable="control_to_sim",
            ),
            Node(
                package="sim_translation",
                executable="map_to_cone_detection",
            ),
            Node(
                package="sim_translation",
                executable="map_to_path",
            ),
            Node(
                package="sim_translation",
                executable="sim_to_odom",
            ),
            Node(
                package="sim_translation",
                executable="sim_to_cam",
            ),
            Node(
                package="sim_translation",
                executable="sim_to_velodyne",
            ),
            Node(
                package="sim_translation",
                executable="sim_transform",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(get_package_share_path("models") / "launch" / "robot_description.launch.py")
                ),
                launch_arguments=[
                    ("urdf_model", "qev3.urdf.xacro"),
                ],
            ),
            # IncludeLaunchDescription(
            #     launch_description_source=PythonLaunchDescriptionSource(
            #         launch_file_path=str(get_package_share_path("driverless_common") / "launch" / "display.launch.py")
            #     ),
            # ),
        ]
    )
