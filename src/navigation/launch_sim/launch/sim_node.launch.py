from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_sensing',
            executable='find_cones',
        ),
        Node(
            package='sim_planning',
            executable='mapper',
        ),
    ])
