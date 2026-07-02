import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    # Process URDF xacro with hardware_interface='mock'
    xacro_path = os.path.join(
        get_package_share_directory("ros2_control_bringup"),
        "urdf",
        "test_qev-3d.urdf.xacro",
    )
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "hardware_interface": "mock",
            "can_interface": "none",
        },
    )
    robot_description = {"robot_description": doc.toprettyxml(indent="  ")}

    # Get controllers configuration
    controllers_file = os.path.join(
        get_package_share_directory("ros2_control_bringup"), "config", "qev-3d_controllers.yaml"
    )

    # Node to run controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
    )

    # Node to publish robot state
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"rate": 200}],
    )

    # Spawner for joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner for ackermann steering direct controller
    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_direct_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        ackermann_steering_spawner,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
