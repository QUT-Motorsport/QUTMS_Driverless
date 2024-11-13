from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    zenoh_node = Node(
        package="zenoh_bridge_ros2dds", executable="zenoh_bridge_ros2dds", condition=IfCondition(use_zenoh_bridge)
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

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            use_zenoh_bridge_arg,
            rosboard_node,
            display_node,
            foxglove_node,
            # zenoh_node,
        ]
    )
