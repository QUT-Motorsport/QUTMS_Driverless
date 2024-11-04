import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    bringup_dir = get_package_share_path("nav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")

    localisation_params_file = os.path.join(bringup_dir, "config", "localisation_params.yaml")
    slam_params_file = os.path.join(bringup_dir, "config", "slam_toolbox_params.yaml")

    # Community ROS 2 packages
    localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="both",
        parameters=[localisation_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "control/nav_cmd_vel"),
            ("/enable", "/odometry/enable"),
            ("/set_pose", "/odometry/set_pose"),
        ],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_node",
        output="both",
        namespace=namespace,
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/slam_toolbox/graph_visualization", "/slam/graph_visualisation"),
            ("/pose", "/slam/pose"),
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node), transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(autostart),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) or recording clock if true",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the stack",
    )

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            declare_use_sim_time_cmd,
            declare_namespace_cmd,
            declare_log_level_cmd,
            declare_autostart_cmd,
            localisation_node,
            slam_toolbox_node,
            configure_event,
            activate_event,
        ]
    )
