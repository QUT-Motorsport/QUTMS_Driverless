# Copyright 2022 Stereolabs
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

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import Node

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

config_common_path = os.path.join(
    get_package_share_directory('sensors'),
    'config',
    'zed_common.yaml'
)
config_camera_path = os.path.join(
    get_package_share_directory('sensors'),
    'config',
    'zed2i.yaml'
)


def launch_setup(context, *args, **kwargs):
    common_params = yaml.load(open(config_common_path))

    # Get parameters from yaml
    base_frame = common_params['/**']['ros__parameters']['pos_tracking']['base_frame']
    cam_pose = common_params['/**']['ros__parameters']['pos_tracking']['initial_base_pose']    

    # Robot State Publisher node
    rsp_node = Node(
        # condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace='zed2i',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', default_xacro_path, ' ',
                    'camera_name:=zed2i ',
                    'camera_model:=zed2i ',
                    'base_frame:=', base_frame, ' ',
                    'cam_pos_x:=', cam_pose[0], ' ',
                    'cam_pos_y:=', cam_pose[1], ' ',
                    'cam_pos_z:=', cam_pose[2], ' ',
                    'cam_roll:=', cam_pose[3], ' ',
                    'cam_pitch:=', cam_pose[4], ' ',
                    'cam_yaw:=', cam_pose[5]
                ])
        }]
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        namespace='zed2i',
        executable='zed_wrapper',
        name='zed_node',
        output='screen',
        #prefix=['xterm -e valgrind --tools=callgrind'],
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
        ]
    )

    return [
        rsp_node,
        zed_wrapper_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            OpaqueFunction(function=launch_setup)
        ]
    )
