import os
 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
 
 
def generate_launch_description():
 
	package_name = 'air_scan_end_effector'

 
	params_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')
	cwd = f'{Path(__file__).parents[5]}/src/{package_name}/config/params.yaml'
	os.system(f'cp {cwd} {params_file}')
 
	air_scan_end_effector = Node(
		package=package_name,
		executable='air_scan_driver',
		parameters=[params_file]
 
	)
 
	return LaunchDescription([
		air_scan_end_effector
	])
