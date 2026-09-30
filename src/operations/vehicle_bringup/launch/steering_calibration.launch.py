# run to generate calibration file
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
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

    calibration_node = Node(
        package="vehicle_bringup",
        executable="steering_calibration_node",
        output="both",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            scs_container,
            calibration_node,
        ]
    )
