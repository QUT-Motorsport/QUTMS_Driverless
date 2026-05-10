from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='rosbag_creator_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='rosbag_creator',
                plugin='rosbag_creator::Rosbag2RecorderComponent',
                name='rosbag_creator'
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
