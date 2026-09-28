#!/usr/bin/env python3
"""Launch the shared simulation environment and RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time')
	world = LaunchConfiguration('world')
	rviz = LaunchConfiguration('rviz')

	return LaunchDescription([
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='True',
			description='Use simulation time for every launched subsystem.',
		),
		DeclareLaunchArgument(
			'world',
			default_value='simple_trees',
			description='Simulation world to load.',
		),
		DeclareLaunchArgument(
			'rviz',
			default_value='True',
			description='Launch RViz with the simulation.',
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(PathJoinSubstitution([
				FindPackageShare('41068_ignition_bringup'),
				'launch',
				'41068_ignition.launch.py',
			])),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'rviz': rviz,
				'world': world,
				'nav2': 'False',
				'slam': 'False',
				'husky': 'True',
				'parrot': 'False',
			}.items(),
		),
	])
