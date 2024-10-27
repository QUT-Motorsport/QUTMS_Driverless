from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # control node
    sine_node = Node(
        package="controllers",
        executable="sine",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            sine_node,
        ]
    )
