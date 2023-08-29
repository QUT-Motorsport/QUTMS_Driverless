from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="mission_launcher",
            ),
            Node(
                package="canbus",
                executable="canbus_translator_node",
                parameters=[
                    get_package_share_path("canbus") / "config" / "canbus.yaml",
                ],
            ),
            Node(
                package="vehicle_supervisor",
                executable="vehicle_supervisor_slim_node",
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            Node(
                package="driverless_common",
                executable="display",
            ),
            Node(
                package="lidar_pipeline",
                executable="lidar_detector_node",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(get_package_share_path("sensors") / "launch" / "vlp32.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(get_package_share_path("sensors") / "launch" / "sbg_device.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py"
                    )
                ),
                launch_arguments=[
                    ("urdf_model", "qev-3d.urdf.xacro"),
                    ("base_frame", "base_footprint"),
                ],
            ),
        ]
    )
