from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    testing = DeclareLaunchArgument(name="testing", default_value="False")

    return LaunchDescription(
        [
            testing,
            Node(package="missions", executable="ebs_test", parameters=[{"testing": LaunchConfiguration("testing")}]),
            # Node(
            #     package="vision_pipeline",
            #     executable="trt_detector",
            # ),
            # Node(
            #     package="lidar_pipeline",
            #     executable="lidar_processing",
            # ),
            Node(
                package="controllers",
                executable="reactive_control",
            ),
        ]
    )
