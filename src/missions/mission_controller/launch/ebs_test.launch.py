from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="ebs_test_mission",
            ),
            Node(
                package="lidar_pipeline_3",
                executable="lidar_perception",
            ),
            # Node(
            #     package="controllers",
            #     executable="reactive_control",
            #     parameters=[{"ebs_control": True}],
            # ),
            Node(
                package="yaw_controller",
                executable="yaw_controller",
                parameters=[
                    get_package_share_path("yaw_controller") / "config" / "yaw_controller.yaml",
                ],
            ),
        ]
    )
