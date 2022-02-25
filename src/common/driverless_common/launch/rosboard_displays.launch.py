"""
launch with `ros2 launch driverless_common rosboard_displays.launch.py`
probably can tab to complete most of it
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosboard',
            executable='rosboard_node',
        ),
        Node(
            package='driverless_common',
            executable='display',
        ),
    ])
