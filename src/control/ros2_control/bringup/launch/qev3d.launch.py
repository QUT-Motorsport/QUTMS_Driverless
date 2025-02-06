from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # region [declare arguments]
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",  # Argument name
            default_value="vehicle_urdf",  # Default argument value
            description="Description package with robot URDF/xacro files. Usually the argument \
            is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",  # Argument name
            default_value="qev-3d.urdf.xacro",  # Default argument value
            description="URDF/xacro file to load.",
        )
    ),
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
            with this launch file.",
        )
    ),
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",  # Argument name
            default_value='""',  # value of argument is empty string
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    ),
    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_odometry_tf",  # Argument name
            default_value="false",  # Default argument value
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )
    # endregion

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("vehicle_urdf"), "urdf", description_file]),
            " ",
            "prefix:=",  # prefix argument is passed to xacro, WHICH IS EMPTY
            prefix,
        ]
    )

    # region [initialize arguments]
    robot_description = ParameterValue(
        robot_description_content, value_type=str
    )  # variable storing the value of the argument
    prefix = LaunchConfiguration("prefix")  # varible storing prefix (an empty string)
    gui = LaunchConfiguration("gui")  # "True" - enable something
    description_file = LaunchConfiguration("description_file")  # storing URDF file name (qev-3d.urdf.xacro)
    description_package = LaunchConfiguration("description_package")  # storing package name (vehicle_urdf)
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")  # "False" - disable something
    # endregion

    # region [Path Join]
    robot_controllers = PathJoinSubstitution(  # join the path of controllers configuration file
        [
            FindPackageShare("qev3d_ros2_control"),
            "config",
            "qev3d_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("vehicle_urdf"),
            "rviz",
            "config.rviz",
        ]
    )

    # endregion

    # TODO: SET UP RVIZ2 CONFIGURATION FILE

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),  # Only launch the node if gui is set to true
    )
    control_node = Node(  # launch the control node - launching the controller manager
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
        condition=IfCondition(gui),  # Only launch the node if gui is set to true
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
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

    drive_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_pid_controller", "--param-file", robot_controllers],
    )

    steering_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_pid_controller", "--param-file", robot_controllers],
    )

    # Delay start of forward_position_controller_spawner after `position_controller_spawner`
    delay_robot_bicycle_controller_spawner_after_steering_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=steering_pid_controller,
            on_exit=[robot_bicycle_controller_spawner],
        )
    )
    delay_robot_bicycle_controller_spawner_after_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_pid_controller,
            on_exit=[robot_bicycle_controller_spawner],
        )
    )
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        ),
    )

    nodes = [
        # joint_state_publisher_node,
        control_node,
        robot_state_pub_bicycle_node,
        joint_state_broadcaster_spawner,
        drive_pid_controller,
        steering_pid_controller,
        delay_robot_bicycle_controller_spawner_after_steering_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
