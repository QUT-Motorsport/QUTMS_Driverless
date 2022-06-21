from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canbus',
            executable='canbus',
        ),
        Node(
            package='rosboard',
            executable='rosboard_node',
        ),
    ])
