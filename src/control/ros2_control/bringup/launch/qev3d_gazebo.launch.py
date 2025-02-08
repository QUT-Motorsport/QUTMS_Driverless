from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            "gui", default_value="false", description="Start Rviz2 and Joint State Publisher GUI automatically."
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

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "qev3d_robot",
            "-allow_renaming",
            "true",
        ],
    )

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
        ],
        condition=IfCondition(remap_odometry_tf),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(gui),  # Only launch the node if gui is set to true
    )

    # Event Handlers for delayed execution
    delay_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_bicycle_node,
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
        gazebo,
        gz_spawn_entity,
        control_node,
        joint_state_publisher_node,
        robot_state_pub_bicycle_node,
        delay_controller_spawners,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
