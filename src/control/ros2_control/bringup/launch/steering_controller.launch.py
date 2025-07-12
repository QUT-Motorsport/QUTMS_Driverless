from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
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
            default_value="true",
            description="Remap odometry TF from the steering controller to the TF tree.",
        ),
    ]

    # Launch configurations
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")
    description_file = LaunchConfiguration("description_file")
    description_package = LaunchConfiguration("description_package")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    # Generate robot description content via xacro
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

    # Path to the robot controller configuration file
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("qev3d_ros2_control"), "config", "qev3d_controller.yaml"]
    )

    # Spawning the bicycle steering controller
    robot_bicycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bicycle_steering_controller",
            "--param-file",
            robot_controllers,
        ],
        condition=UnlessCondition(remap_odometry_tf),
    )
    passthrough_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["passthrough_controller", "--param-file", robot_controllers],
    )

    robot_bicycle_controller_spawner_remapped = Node(
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

    delay_bicycle_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=passthrough_controller,
            on_start=[
                robot_bicycle_controller_spawner,
                robot_bicycle_controller_spawner_remapped,
            ],
        )
    )
    # Node for robot state publisher (publish robot description)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Add nodes to launch list
    nodes = [
        # robot_state_publisher,
        robot_bicycle_controller_spawner,
        robot_bicycle_controller_spawner_remapped,
    ]

    # If GUI is enabled, start Rviz and Joint State Publisher
    if gui == "true":
        rviz2_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("vehicle_urdf"), "rviz", "config.rviz"])],
        )
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
        nodes.extend([rviz2_node, joint_state_publisher_gui])

    return LaunchDescription(declared_arguments + nodes)
