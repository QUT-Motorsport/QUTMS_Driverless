from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
                package="vehicle_supervisor",
                plugin="vehicle_supervisor::ASSupervisor",
                name="vehicle_supervisor_node",
                parameters=[
                    {"manual_override": False},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
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
                package="velocity_controller",
                plugin="velocity_controller::VelocityController",
                name="velocity_controller_node",
                parameters=[
                    get_package_share_path("velocity_controller") / "config" / "velocity_controller.yaml",
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

    return LaunchDescription(
        [
            scs_container,
            Node(
                package="rosboard",
                executable="rosboard_node",
            ),
            # Node(
            #     package="foxglove_bridge",
            #     executable="foxglove_bridge",
            #     output="screen",
            #     parameters=[
            #         {"port": 8765},
            #         {"address": "0.0.0.0"},
            #         {"tls": False},
            #         {"certfile": ""},
            #         {"keyfile": ""},
            #         {"topic_whitelist": [".*"]},
            #         {"param_whitelist": [".*"]},
            #         {"service_whitelist": [".*"]},
            #         {"client_topic_whitelist": [".*"]},
            #         {"min_qos_depth": 1},
            #         {"max_qos_depth": 10},
            #         {"num_threads": 0},
            #         {"send_buffer_limit": 1000000000},
            #         {"use_sim_time": False},
            #         {
            #             "capabilities": [
            #                 "clientPublish",
            #                 "parameters",
            #                 "parametersSubscribe",
            #                 "services",
            #                 "connectionGraph",
            #                 "assets",
            #             ]
            #         },
            #         {"include_hidden": False},
            #         {
            #             "asset_uri_allowlist": [
            #                 "^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
            #             ]
            #         },
            #     ],
            # ),
            Node(
                package="driverless_common",
                executable="display",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("lidar_pipeline") / "launch" / "lidar_pipeline.launch.py"
                    )
                ),
            ),
            Node(
                package="mission_controller",
                executable="mission_launcher_node",
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
                    ("display_car", "false"),
                ],
            ),
        ]
    )
