from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # safety critical signals should use intra process comms for low latency
    scs_container = ComposableNodeContainer(
        name="critcial_signal_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="steering_actuator",
                plugin="steering_actuator::SteeringActuator",
                name="steering_actuator_node",
                parameters=[
                    get_package_share_path("steering_actuator") / "config" / "steering.yaml",
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # ComposableNode(
            #     package="velocity_controller",
            #     plugin="velocity_controller::VelocityController",
            #     name="velocity_controller_node",
            #     parameters=[
            #         get_package_share_path("velocity_controller") / "config" / "velocity_controller.yaml",
            #     ],
            #     extra_arguments=[{"use_intra_process_comms": True}],
            # ),
            ComposableNode(
                package="canbus",
                plugin="canbus::CANTranslator",
                name="canbus_translator_node",
                parameters=[
                    get_package_share_path("canbus") / "config" / "canbus.yaml",
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    display_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("vehicle_bringup") / "launch" / "displays.launch.py")
        ),
        launch_arguments=[
            ("use_zenoh_bridge", "false"),
        ],
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

    perception_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("perception_bringup") / "launch" / "perception.launch.py")
        ),
    )

    rosbag_creator_node = Node(
        package="rosbag_creator",
        executable="rosbag_creator_node",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            # scs_container,
            display_launch,
            urdf_launch,
            perception_launch,
            rosbag_creator_node,
        ]
    )
