from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                "/home/alistair/dev/repos/QUTMS_Driverless/src/models/models/urdf/qev3.urdf.xacro",
                            ]
                        )
                    }
                ],
            ),
        ]
    )
