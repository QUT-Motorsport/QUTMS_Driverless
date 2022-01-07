from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TODO: include sensors launch file
        Node(
            package='rosboard',
            executable='rosboard_node',
        ),
        Node(
            package='controllers',
            executable='simple',
        ),
    ])
