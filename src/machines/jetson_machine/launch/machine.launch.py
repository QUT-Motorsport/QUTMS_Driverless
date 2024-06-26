from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vision_detector_node = Node(
        package="vision_pipeline",
        executable="v8_torch_detector",
        parameters=[{"debug_bbox": True}, {"log_level": "INFO"}],
    )

    zed_camera_driver = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(get_package_share_path("sensors") / "launch" / "zed_camera.launch.py")
        ),
    )

    return LaunchDescription(
        [
            vision_detector_node,
            zed_camera_driver,
        ]
    )
