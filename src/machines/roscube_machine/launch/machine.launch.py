from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="canbus",
            executable="canbus",
            parameters=["/home/developer/driverless_ws/src/hardware/canbus/config/canbus.yaml"],  # TODO: use auto getter for path
        ),
        Node(
            package="rosboard",
            executable="rosboard_node",
        ),
        IncludeLaunchDescription(
            package="sensors",
            launch="sensors.launch.py"
        ),
    ])
