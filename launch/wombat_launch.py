#!/usr/bin/env python3
"""Launch the simple direct-drive Wombat mission stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation time.'),
        DeclareLaunchArgument(
            'namespace', default_value='wombat',
            description='Namespace for the simple mission stack.'),
        Node(
            package='rs1',
            executable='mission_node',
            name='mission_node',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rs1',
            executable='wombat_node',
            name='wombat_node',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])