from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
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
            default_value="true",
            description="Remap odometry TF from the steering controller to the TF tree.",
        ),
    ]

    # Initialize arguments
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")
    description_file = LaunchConfiguration("description_file")

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
    robot_state_publisher_spawner = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": True, "initial_positions": 0.0}],
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

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(gui),  # Only launch the node if gui is set to true
    )

    # Event Handlers for delayed execution
    delay_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_spawner,
            on_start=[
                joint_state_broadcaster_spawner,
                drive_pid_controller,
                steering_pid_controller,
            ],
        )
    )
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[rviz_node],
        )
    )

    # Nodes to launch
    nodes = [
        control_node,
        joint_state_publisher_node,
        robot_state_publisher_spawner,
        delay_controller_spawners,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
