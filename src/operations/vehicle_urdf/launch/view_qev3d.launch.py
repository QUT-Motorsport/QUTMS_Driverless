from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",  # Argument name
            default_value="vehicle_urdf",  # Default argument value
            description="Description package with robot URDF/xacro files. Usually the argument \
            is not set, it enables use of a custom description.",
        )
    )

    # initialize the variable storing the package name
    description_package = LaunchConfiguration("description_package")

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",  # Argument name
            default_value="qev-3d.urdf.xacro",  # Default argument value
            description="URDF/xacro file to load.",
        )
    ),

    # initialize the variable storing the file name
    description_file = LaunchConfiguration("description_file")

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
            with this launch file.",
        )
    ),

    # initialize the variable storing the value of the argument
    gui = LaunchConfiguration("gui")

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",  # Argument name
            default_value='""',  # value of argument is empty string
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    ),

    # initialize the variable storing the value of the argument
    prefix = LaunchConfiguration("prefix")

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
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # TODO: SET UP RVIZ2 CONFIGURATION FILE
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "robot.rviz"])

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),  # Only launch the node if gui is set to true
    )
    robot_state_publisher_node = Node(
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

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
