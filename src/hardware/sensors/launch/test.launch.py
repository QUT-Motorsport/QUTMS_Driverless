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
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import Node

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
    common_params = yaml.safe_load(open(config_common_path))

    # Get parameters from yaml
    base_frame = common_params['/**']['ros__parameters']['pos_tracking']['base_frame']
    cam_pose = common_params['/**']['ros__parameters']['pos_tracking']['initial_base_pose']
    print(cam_pose)
    print(base_frame)

def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            OpaqueFunction(function=launch_setup)
        ]
    )
