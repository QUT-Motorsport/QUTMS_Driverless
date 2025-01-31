from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
      # List of arguments to be declared into Node  
      declared_arguments = []

      # Open Rviz2 GUI
      declared_arguments.append(
            DeclareLaunchArgument(
                  "gui",
                  default_value="true", # Default value is true
                  description="Start RViz2 automatically with this launch file.",
            )
      )

      # Remap odometry TF from the steering controller to the TF tree
      declared_arguments.append( 
            DeclareLaunchArgument(
                  "remap_odometry_tf",
                  default_value="false", # Default value is false
                  description="Remap odometry TF from the steering controller to the TF tree.",
            )
      )

      # Arguments Initialization
      gui = LaunchConfiguration("gui")
      remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")