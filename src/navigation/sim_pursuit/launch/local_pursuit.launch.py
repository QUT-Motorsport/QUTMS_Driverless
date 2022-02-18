"""
launch with `ros2 launch sim_pursuit local_pursuit.launch.py`
probably can tab to complete most of it
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_pursuit',
            executable='detection',
        ),
        Node(
            package='sim_pursuit',
            executable='local_pursuit',
        ),
    ])
