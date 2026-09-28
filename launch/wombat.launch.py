#!/usr/bin/env python3
"""Launch Nav2 and the Wombat mission stack for one robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    global_frame = LaunchConfiguration('global_frame')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    nav2 = LaunchConfiguration('nav2')
    slam = LaunchConfiguration('slam')

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
        DeclareLaunchArgument(
            'nav2', default_value='True',
            description='Launch Nav2 for the Wombat robot.'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='Launch SLAM Toolbox independently of Nav2.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch',
                '41068_navigation.launch.py',
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'config_filename_suffix': '_husky1',
                'nav2': nav2,
                'slam': slam,
            }.items(),
        ),
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