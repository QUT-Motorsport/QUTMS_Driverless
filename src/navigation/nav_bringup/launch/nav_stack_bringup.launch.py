# Copyright (c) 2024 Open Navigation LLC
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav_bringup")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(source_file=params_file, root_key="", param_rewrites=param_substitutions, convert_types=True),
        allow_substs=True,
    )

    # Specify the actions
    nav_container = Node(
        name="nav2_container",
        package="rclcpp_components",
        executable="component_container_isolated",
        parameters=[configured_params, {"use_sim_time": use_sim_time, "autostart": autostart}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "localisation.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "autostart": autostart, "namespace": namespace}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "navigation.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "log_level": log_level, "namespace": namespace}.items(),
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

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level",
    )

    # Create the launch description and populate
    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            declare_use_sim_time_cmd,
            declare_namespace_cmd,
            declare_autostart_cmd,
            declare_log_level_cmd,
            nav_container,
            localisation_launch,
            navigation_launch,
        ]
    )
