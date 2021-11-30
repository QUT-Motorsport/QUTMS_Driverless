#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_common_path = os.path.join(
        get_package_share_directory("sensors"),
        "config",
        "zed",
        "common.yaml"
    )

    config_camera_path = os.path.join(
        get_package_share_directory("sensors"),
        "config",
        "zed",
        "zed2i.yaml"
    )

    zed_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/zed2i_camera.launch.py"
            ]
        ),
        launch_arguments={
            "config_common_path": config_common_path,
            "config_camera_path": config_camera_path,
        }
    )

    return LaunchDescription([zed_launch])
