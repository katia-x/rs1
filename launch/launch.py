#!/usr/bin/env python3
"""Launch the complete RS1 system from one composition point."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	# Every subsystem included below receives this shared simulation-time value.
	# Future team members can add another launch include without modifying the
	# Wombat-specific launch file.
	use_sim_time = LaunchConfiguration('use_sim_time')
	namespace = 'wombat'
	package_share = FindPackageShare('rs1')

	# This list is the integration point for the complete RS1 system. Keeping
	# each subsystem in its own file preserves clear ownership as it grows.
	subsystem_launches = [
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(PathJoinSubstitution([
				package_share, 'launch', 'wombat_launch.py'
			])),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'namespace': namespace,
			}.items(),
		),
	]

	return LaunchDescription([
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='True',
			description='Use simulation time for every launched subsystem.',
		),
		*subsystem_launches,
	])
