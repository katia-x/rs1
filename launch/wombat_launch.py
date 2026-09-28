#!/usr/bin/env python3
"""Launch the simple direct-drive Wombat mission stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    global_frame = LaunchConfiguration('global_frame')
    robot_base_frame = LaunchConfiguration('robot_base_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation time.'),
        DeclareLaunchArgument(
            'namespace', default_value='husky1',
            description='Namespace for the simple mission stack.'),
        DeclareLaunchArgument(
            'global_frame', default_value='husky1_map',
            description='Global frame used by Mission and Nav2.'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='husky1_base_link',
            description='Robot base frame used by Mission.'),
        Node(
            package='rs1',
            executable='mission_node',
            name='mission_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'global_frame': global_frame,
                'wombat_base_frame': robot_base_frame,
            }],
        ),
        Node(
            package='rs1',
            executable='wombat_node',
            name='wombat_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'global_frame': global_frame,
            }],
        ),
    ])