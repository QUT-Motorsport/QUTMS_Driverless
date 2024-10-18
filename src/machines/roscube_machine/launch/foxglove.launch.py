
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                output="screen",
                parameters=[
                    {"port": 8765},
                    {"address": "0.0.0.0"},
                    {"tls": False},
                    {"certfile": ""},
                    {"keyfile": ""},
                    {"topic_whitelist": [".*"]},
                    {"param_whitelist": [".*"]},
                    {"service_whitelist": [".*"]},
                    {"client_topic_whitelist": [".*"]},
                    {"min_qos_depth": 1},
                    {"max_qos_depth": 10},
                    {"num_threads": 0},
                    {"send_buffer_limit": 10000000},
                    {"use_sim_time": False},
                    {
                        "capabilities": [
                            "clientPublish",
                            "parameters",
                            "parametersSubscribe",
                            "services",
                            "connectionGraph",
                            "assets",
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
        ]
    )
