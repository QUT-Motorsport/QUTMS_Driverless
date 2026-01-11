from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)


def load_zenoh_node(context, *args, **kwargs):
    zenoh_ip = get_argument(context, "zenoh_ip")
    zenoh_port = get_argument(context, "zenoh_port")

    print(f"zenoh_ip: {zenoh_ip}")
    print(f"zenoh_port: {zenoh_port}")

    return [
        Node(
            package="zenoh_bridge_ros2dds",
            executable="zenoh_bridge_ros2dds",
            arguments=["-e", f"tcp/{zenoh_ip}:{zenoh_port}"],
            output="both",
        )
    ]


def generate_launch_description():

    zenoh_ip_arg = DeclareLaunchArgument("zenoh_ip", default_value="192.168.3.4")
    zenoh_port_arg = DeclareLaunchArgument("zenoh_port", default_value="7447")

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
    )

    return LaunchDescription(
        [
            zenoh_ip_arg,
            zenoh_port_arg,
            foxglove_bridge,
            OpaqueFunction(function=load_zenoh_node),
        ]
    )
