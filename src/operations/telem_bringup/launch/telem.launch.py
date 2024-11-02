from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    zenoh_ip = LaunchConfiguration("zenoh_ip")
    zenoh_port = LaunchConfiguration("zenoh_port")

    zenoh_ip_arg = DeclareLaunchArgument("zenoh_ip", default_value="192.168.3.4")
    zenoh_port_arg = DeclareLaunchArgument("zenoh_port", default_value="7447")

    zenoh_bridge = Node(
        package="zenoh_bridge_ros2dds",
        executable="zenoh_bridge_ros2dds",
        arguments=["-e", f"tcp/{zenoh_ip}:{zenoh_port}"],
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
    )

    return LaunchDescription([zenoh_ip_arg, zenoh_port_arg, zenoh_bridge, foxglove_bridge])
