# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_spawners(
    context: LaunchContext,
    controller_config: LaunchConfiguration,
    inactive_controller_config: LaunchConfiguration,
    joint_state_broadcaster_spawner: Node,
) -> list:
    # Substitiutes LaunchConfigurations into str
    controller_config = context.perform_substitution(controller_config)
    inactive_controller_config = context.perform_substitution(inactive_controller_config)

    spawner_return = []

    # Evaluates the controller names config to determine if controllers are to be spawned individually or as a chain.
    # Makes sure that the configuration is not empty, otherwise the spawners should be skipped
    robot_controller_names = []
    if not (controller_config == "" or controller_config == "[]"):
        eval("robot_controller_names.append(" + controller_config + ")")
    robot_controller_spawners = []

    for controller in robot_controller_names:
        if type(controller) is str:
            robot_controller_spawners += [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[controller, "-c", "/controller_manager"],
                )
            ]
        elif type(controller) is list:
            controller_spawner_args = ["-c", "/controller_manager", "--activate-as-group"]
            controller_spawner_args.extend(controller)
            robot_controller_spawners += [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=controller_spawner_args,
                )
            ]
        else:
            raise TypeError(
                f"Controller {controller} input not recognized. \
                    str or list expected but received {type(controller)}"
            )

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(robot_controller_spawners[i - 1] if i > 0 else joint_state_broadcaster_spawner),
                    on_exit=[controller],
                )
            )
        ]
    spawner_return.extend(delay_robot_controller_spawners_after_joint_state_broadcaster_spawner)

    # Repeats the process for inactive spawners,
    # but only if at least one active controller is configured
    inactive_robot_controller_names = []
    if (not (inactive_controller_config == "" or inactive_controller_config == "[]")) and len(
        robot_controller_spawners
    ) > 0:
        eval("inactive_robot_controller_names.append(" + inactive_controller_config + ")")
    inactive_robot_controller_spawners = []

    for controller in inactive_robot_controller_names:
        if type(controller) is str:
            inactive_robot_controller_spawners += [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[controller, "-c", "/controller_manager", "--inactive"],
                )
            ]
        elif type(controller) is list:
            inactive_controller_spawner_args = ["-c", "/controller_manager", "--inactive", "--activate-as-group"]
            inactive_controller_spawner_args.extend(controller)
            inactive_robot_controller_spawners += [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=inactive_controller_spawner_args,
                )
            ]
        else:
            raise TypeError(
                f"Inactive controller {controller} input not recognized. \
                    str or list expected but received {type(controller)}"
            )

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1] if i > 0 else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]
    spawner_return.extend(delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner)
    return spawner_return


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ros2_control_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="qev-3d_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="vehicle_urdf",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="qev-3d.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value='["steering_pid_controller", "drive_pid_controller", "ackermann_steering_controller"]',
            description="Robot controller to start. Pass string to start individual \
                controllers or python list to start in chain mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "inactive_robot_controller",
            default_value="",
            description="Inactive robot controller to start. Pass string to start individual \
                controllers or python list to start in chain mode",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    inactive_robot_controller = LaunchConfiguration("inactive_robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "qev-3d.rviz"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay loading and activation of `ros2_control_node` after start of robot_state_pub_node
    delay_ros2_control_node_spawner_after_robot_state_pub_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[control_node],
                ),
            ],
        )
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            rviz_node,
            delay_ros2_control_node_spawner_after_robot_state_pub_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            OpaqueFunction(
                function=launch_spawners,
                args=[robot_controller, inactive_robot_controller, joint_state_broadcaster_spawner],
            ),
        ]
    )
