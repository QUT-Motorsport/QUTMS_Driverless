from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="vehicle_urdf",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="qev-3d.urdf.xacro", description="URDF/xacro file to load."
        ),
        DeclareLaunchArgument(
            "gui", default_value="true", description="Start Rviz2 and Joint State Publisher GUI automatically."
        ),
        DeclareLaunchArgument(
            "prefix", default_value='""', description="Prefix of the joint names, useful for multi-robot setup."
        ),
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="false",
            description="Remap odometry TF from the steering controller to the TF tree.",
        ),
    ]

    # Initialize arguments
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")
    description_file = LaunchConfiguration("description_file")
    description_package = LaunchConfiguration("description_package")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("vehicle_urdf"), "urdf", description_file]),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Path configurations
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("qev3d_ros2_control"), "config", "qev3d_controller.yaml"]
    )
    rviz_config_file = PathJoinSubstitution([FindPackageShare("vehicle_urdf"), "rviz", "config.rviz"])

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_bicycle_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    drive_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_pid_controller", "--param-file", robot_controllers],
    )
    steering_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_pid_controller", "--param-file", robot_controllers],
    )
    robot_bicycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bicycle_steering_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /bicycle_steering_controller/tf_odometry:=/tf",
        ],
        condition=IfCondition(remap_odometry_tf),
    )

    # Event Handlers for delayed execution
    delay_robot_bicycle_controller_spawner_after_steering_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=steering_pid_controller,
            on_exit=[robot_bicycle_controller_spawner],
        )
    )
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Nodes to launch
    nodes = [
        control_node,
        robot_state_pub_bicycle_node,
        joint_state_broadcaster_spawner,
        drive_pid_controller,
        steering_pid_controller,
        delay_robot_bicycle_controller_spawner_after_steering_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
