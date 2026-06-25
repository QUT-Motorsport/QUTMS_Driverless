import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    urdf_model = LaunchConfiguration("urdf_model").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    display_car = LaunchConfiguration("display_car").perform(context)

    # Process URDF xacro
    xacro_path = os.path.join(
        get_package_share_directory("vehicle_urdf"),
        "urdf",
        urdf_model,
    )
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "base_frame": base_frame,
            "display_car": display_car,
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

    # Spawner for Ackermann steering/traction controller
    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Base frame of the vehicle",
            ),
            DeclareLaunchArgument(
                "urdf_model",
                default_value="qev-3d.urdf.xacro",
                description="URDF Model to use (from vehicle_urdf/urdf)",
            ),
            DeclareLaunchArgument(
                "display_car",
                default_value="true",
                description="Display the car in rviz",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
