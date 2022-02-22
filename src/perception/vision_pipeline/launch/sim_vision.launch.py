


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_pipeline',
            executable='zed_simulator',
        ),
        Node(
            package='vision_pipeline',
            executable='torch_detector',
        ),
    ])
