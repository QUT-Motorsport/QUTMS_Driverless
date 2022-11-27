from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_pipeline",
                executable="torch_detector",
            ),
            Node(
                package="lidar_pipeline_3",
                executable="lidar_perception",
            ),
            Node(
                package="py_slam",
                executable="slam",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(get_package_share_path("models") / "launch" / "robot_description.launch.py")
                ),
                launch_arguments=[
                    ("urdf_model", "qev3.urdf.xacro"),
                ],
            ),
        ]
    )
