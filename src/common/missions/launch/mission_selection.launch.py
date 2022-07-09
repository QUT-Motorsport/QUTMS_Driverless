# # need launch file to start mission selector
# # launch file calls other launch files for the specific mission

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.substitutions import FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"

    mission_node = Node(
        package='missions',
        executable='mission_control',
        env=node_env,
    )   
 
    mission_pkg = get_package_share_directory('missions')

    return LaunchDescription([
        mission_node,
        RegisterEventHandler(
            OnProcessIO(
                target_action=mission_node,
                on_stdout=lambda event: IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([mission_pkg,"/",event.text.decode().strip(),".launch.py"]),
                )
            )
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
