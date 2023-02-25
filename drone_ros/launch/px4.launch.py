#!/usr/bin/env python3
"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	"""Launch Gazebo with a drone running PX4 communicating over ROS 2."""
	HOME = os.environ.get('HOME')
	PX4_RUN_DIR = HOME + '/tmp/px4_run_dir'
	gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

	blackdrones_description_dir = get_package_share_directory('blackdrones_description')
	world = os.path.join(blackdrones_description_dir, 'worlds', 'blackdrone.world')
	model = os.path.join(blackdrones_description_dir, 'models', 'blackdrone', 'blackdrone.urdf')
	custom_gazebo_models = os.path.join(blackdrones_description_dir, 'models')
	px4_init = os.path.join(blackdrones_description_dir, 'PX4-init')

	os.makedirs(PX4_RUN_DIR, exist_ok=True)

	return LaunchDescription([
		SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
							   HOME + '/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
		SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/sitl_gazebo/models'
							   + ':' + custom_gazebo_models),

		SetEnvironmentVariable('PX4_SIM_MODEL', 'blackdrone_gimbal'),

		DeclareLaunchArgument('world', default_value=world),
		DeclareLaunchArgument('model', default_value=model),
		DeclareLaunchArgument('x', default_value='1.01'),
		DeclareLaunchArgument('y', default_value='0.98'),
		DeclareLaunchArgument('z', default_value='0.83'),
		DeclareLaunchArgument('R', default_value='0.0'),
		DeclareLaunchArgument('P', default_value='0.0'),
		DeclareLaunchArgument('Y', default_value='0.0'),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
			launch_arguments={'world': LaunchConfiguration('world'),
							  'verbose': 'true'}.items(),
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
		),

		ExecuteProcess(
			cmd=[
				'gz', 'model',
				'--spawn-file', LaunchConfiguration('model'),
				'--model-name', 'drone',
				'-x', LaunchConfiguration('x'),
				'-y', LaunchConfiguration('y'),
				'-z', LaunchConfiguration('z'),
				'-R', LaunchConfiguration('R'),
				'-P', LaunchConfiguration('P'),
				'-Y', LaunchConfiguration('Y')
			],
			prefix="bash -c 'sleep 5s; $0 $@'",
			output='screen'),

		ExecuteProcess(
			cmd=[
				'xterm',
				'-e',
				HOME + '/blackdrones_ws/PX4-Autopilot/build/px4_sitl_rtps/bin/px4',
				px4_init,
				'-s', px4_init + '/init.d-posix/rcS'
			],
			cwd=PX4_RUN_DIR,
			output='screen'),
		ExecuteProcess(
			cmd=['micrortps_agent', '-t', 'UDP'],
			output='screen'),
		ExecuteProcess(
			cmd=['QGroundControl'],
			output='screen'),
	])