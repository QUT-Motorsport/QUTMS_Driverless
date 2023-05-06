import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	config = os.path.join(
		get_package_share_directory('sensors'),
		'config',
		'ellipse_N.yaml'
	)

	return LaunchDescription([
		Node(
			package='sbg_driver',
		#	name='sbg_device_1',
			executable = 'sbg_device_mag',
			output = 'screen',
			parameters = [config]
		)
	])
