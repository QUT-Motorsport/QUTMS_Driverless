from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ts_controller_node = Node(
        package="tractive_system_controller",
        executable="tractive_system_controller",
    )
    return LaunchDescription([ts_controller_node])

if __name__ == "__main__":
    generate_launch_description()
