from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="trackdrive_mission",
            ),
            Node(
                package="controllers",
                executable="reactive_control",
            ),
            Node(
                package="path_follower",
                executable="pure_pursuit",
            ),
            Node(
                package="py_slam",
                executable="sbg_slam",
            ),
            Node(
                package="planners",
                executable="ordered_mid_spline",
            ),
            # Node(
            #     package="steering_actuator",
            #     executable="steering_actuator_node",
            #     parameters=[
            #         get_package_share_path("steering_actuator") / "config" / "steering.yaml",
            #     ],
            # ),
            # Node(
            #     package="velocity_controller",
            #     executable="velocity_controller_node",
            #     parameters=[
            #         get_package_share_path("velocity_controller") / "config" / "velocity_controller.yaml",
            #     ],
            # ),
        ]
    )
