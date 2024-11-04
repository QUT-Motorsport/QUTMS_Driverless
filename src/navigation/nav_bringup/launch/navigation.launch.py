# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")

    # behaviour tree xml file location
    # uncomment the XML you want to test
    to_pose_bt_xml = os.path.join(bringup_dir, "behaviour_trees", "follow_path.xml")
    through_poses_bt_xml = os.path.join(bringup_dir, "behaviour_trees", "follow_path.xml")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "default_nav_to_pose_bt_xml": to_pose_bt_xml,
        "default_nav_through_poses_bt_xml": through_poses_bt_xml,
    }

    params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file, root_key=namespace, param_rewrites=param_substitutions, convert_types=True
        ),
        allow_substs=True,
    )

    # other constants
    lifecycle_nodes = [
        "controller_server",
        #    'planner_server',
        "behavior_server",
        "bt_navigator",
    ]

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("cmd_vel", "control/nav_cmd_vel"),
        ("plan", "planning/midline_path"),
        ("map", "slam/occupancy_grid"),
    ]

    # Add nodes to container
    load_composable_nodes = LoadComposableNodes(
        target_container="nav2_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[{"use_sim_time": use_sim_time, "autostart": autostart, "node_names": lifecycle_nodes}],
            ),
        ],
        output="both",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) or recording clock if true",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # Create the launch description and populate
    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            declare_use_sim_time_cmd,
            declare_namespace_cmd,
            declare_autostart_cmd,
            load_composable_nodes,
        ]
    )
