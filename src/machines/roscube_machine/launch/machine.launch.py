from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="canbus",
                executable="canbus",
                parameters=[
                    get_package_share_path("canbus") / "config" / "canbus.yaml",
                ],
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            Node(
                package="steering_actuator",
                executable="steering_actuator",
                parameters=[
                    get_package_share_path("steering_actuator") / "config" / "steering.yaml",
                ],
            ),
            Node(
                package="vehicle_supervisor",
                executable="vehicle_supervisor",
            ),
            # MISSION CONTROL NODE HERE
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(get_package_share_path("sensors") / "launch" / "sensors.launch.py")
                ),
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
