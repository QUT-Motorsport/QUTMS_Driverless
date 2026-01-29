import os.path as path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share_dir = get_package_share_path("vehicle_bringup")
    use_zenoh_bridge = LaunchConfiguration("use_zenoh_bridge")

    use_zenoh_bridge_arg = DeclareLaunchArgument("use_zenoh_bridge", default_value="True")

    rosboard_node = Node(
        package="rosboard",
        executable="rosboard_node",
    )

    display_node = Node(
        package="driverless_common",
        executable="display",
    )

    trackdrive_node = Node(
        package="vehicle_bringup",
        executable="trackdrive_handler_node",
    )

    trackdrive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(get_package_share_path("vehicle_bringup"), "mission_launch", "trackdrive.launch.py"))
    )

    zenoh_node = Node(
        package="zenoh_bridge_ros2dds",
        executable="zenoh_bridge_ros2dds",
        condition=IfCondition(use_zenoh_bridge),
        arguments=["-c", path.join(bringup_share_dir, "config", "zenoh_config.json5")],
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        condition=UnlessCondition(use_zenoh_bridge),
        parameters=[
            {"port": 8765},
            {"address": "0.0.0.0"},
            {"tls": False},
            {"certfile": ""},
            {"keyfile": ""},
            # {
            #     "topic_whitelist": [
            #         "/rosout",
            #         "/tf",
            #         "/tf_static",
            #         "/diagnostics",
            #         "/control/driving_command",
            #         "/system/as_status",
            #         "/slam/occupancy_grid",
            #         "/vehicle/velocity",
            #         "/vehicle/steering_angle",
            #         "/debug_markers/slam_markers",
            #         "/debug_markers/lidar_markers",
            #         "/planning/yellow_bounds",
            #         "/planning/blue_bounds",
            #         "/planning/midline_path",
            #         "/slam/graph_visualisation",
            #         "/local_costmap/published_footprint",
            #     ]
            # },
            {"param_whitelist": [".*"]},
            {"service_whitelist": [".*"]},
            {"client_topic_whitelist": [".*"]},
            {"min_qos_depth": 1},
            {"max_qos_depth": 10},
            {"num_threads": 0},
            {"send_buffer_limit": 10000000000},
            {"use_sim_time": False},
            # {"use_sim_time": True},
            {
                "capabilities": [
                    # "clientPublish",
                    # "parameters",
                    # "parametersSubscribe",
                    "services",
                    # "connectionGraph",
                    # "assets",
                ]
            },
            {"include_hidden": False},
            {
                "asset_uri_allowlist": [
                    "^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
                ]
            },
        ],
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(get_package_share_path("nav_bringup"), "launch", "sbg.launch.py"))
    )

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

    joy_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("teleop_twist_joy") / "launch" / "teleop_launch.py")
        ),
        launch_arguments=[("joy_config", "xbox"), ("scale_angular.yaw", "1.0")],
    )

    twist_to_ackermann_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(
                get_package_share_path("pushcart_cmdvel_to_ackermann") / "launch" / "cmdvel_to_ackermann.launch.py"
            )
        ),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    return LaunchDescription(
        [
            # stdout_linebuf_envvar,
            # use_zenoh_bridge_arg,
            # rosboard_node,
            # display_node,
            # foxglove_node,
            # zenoh_node,
            # sbg_launch,
            scs_container,
            trackdrive_node,
            trackdrive_launch,
            # urdf_launch,
            # perception_launch,
            # rosbag_creator_node,
            # joy_launch,
            # twist_to_ackermann_launch,
        ]
    )
