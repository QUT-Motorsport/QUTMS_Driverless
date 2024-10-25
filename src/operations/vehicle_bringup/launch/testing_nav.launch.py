from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    display_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("vehicle_bringup") / "displays.launch.py")
        ),
    )

    urdf_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py")
        ),
        launch_arguments=[
            ("urdf_model", "qev-3d.urdf.xacro"),
            ("base_frame", "base_footprint"),
            ("display_car", "false"),
        ],
    )

    sbg_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("nav_bringup") / "launch" / "sbg.launch.py")
        ),
    )

    return LaunchDescription(
        [
            display_launch,
            urdf_launch,
            sbg_launch,
        ]
    )
