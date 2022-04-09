"""
launch with `ros2 launch sim_planning map_pursuit.launch.py`
probably can tab to complete most of it
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_planning',
            executable='mapper',
        ),
        Node(
            package='sim_planning',
            executable='pursuit',
        ),
    ])
