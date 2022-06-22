from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            package='velodyne',
            launch='velodyne-all-nodes-VLP32C-launch.py'
        ),
    ])
