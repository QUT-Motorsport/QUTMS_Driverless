from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ts_controller_node = Node(
        package="tractive_system_controller",
        executable="tractive_system_controller",
    )
    steering_node = Node(
        package="steering_actuator",
        executable="steering_actuator",
    )
    return LaunchDescription(
        [
            ts_controller_node,
            steering_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
