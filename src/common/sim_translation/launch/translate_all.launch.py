from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sim_translation",
                executable="control_to_sim",
            ),
            Node(
                package="sim_translation",
                executable="map_to_cone_detection",
            ),
            Node(
                package="sim_translation",
                executable="map_to_path",
            ),
            Node(
                package="sim_translation",
                executable="sim_to_odom",
            ),
            Node(
                package="sim_translation",
                executable="sim_to_cam",
            ),
            Node(
                package="sim_translation",
                executable="sim_to_velodyne",
            ),
        ]
    )
