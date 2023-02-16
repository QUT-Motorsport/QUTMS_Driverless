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
            # Node(
            #     package="steering_actuator",
            #     executable="steering",
            #     parameters=[
            #         get_package_share_path("steering_actuator") / "config" / "steering.yaml",
            #     ],
            # ),
            Node(
                package="velocity_controller",
                executable="controller",
                parameters=[
                    get_package_share_path("velocity_controller") / "config" / "velocity.yaml",
                ],
            ),
            Node(
                package="vehicle_supervisor",
                executable="vehicle_supervisor",
            ),
            # Node(
            #     package="car_status",
            #     executable="car_status_node",
            # ),
            Node(
                package="driverless_common",
                executable="display",
            ),
            # Node(
            #     package="lidar_pipeline_3",
            #     executable="lidar_perception",
            # ),
            # Node(
            #     package="mission_controller",
            #     executable="mission_control",
            # ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("roscube_machine") / "launch" / "sensor_drivers.launch.py"
                    )
                ),
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py"
                    )
                ),
                launch_arguments=[
                    ("urdf_model", "qev3.urdf.xacro"),
                ],
            ),
        ]
    )
