#!/usr/bin/env python3
"""Launch Wombat autonomy with a self-contained namespaced Nav2 stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Use one shared namespace for the complete RS1 system. Additional system
    # components can join this namespace from the global launch file later.
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    config_path = PathJoinSubstitution([
        FindPackageShare('rs1'), 'config'
    ])

    # This mirrors the example's namespaced SLAM setup while loading the local
    # configuration files from this package.
    slam = GroupAction(scoped=True, actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch',
                'online_async_launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': PathJoinSubstitution([
                    config_path, 'wombat_slam_params.yaml'
                ]),
                'use_lifecycle_manager': 'True',
                'autostart': 'True',
            }.items(),
        ),
    ])

    # Nav2 receives the same namespace and TF/map remappings as SLAM. The local
    # parameter file keeps this launch independent of the example package.
    nav2 = GroupAction(scoped=True, actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch',
            'navigation_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                config_path, 'wombat_nav2_params.yaml'
            ]),
        }.items(),
        ),
    ])

    # Start the planner in the same namespace so its relative action client
    # resolves to the Nav2 server launched above.
    wombat = GroupAction(actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='rs1',
            executable='wombat_node',
            name='wombat_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation time.'),
        DeclareLaunchArgument(
            'namespace', default_value='wombat',
            description='Namespace containing the complete RS1 system.'),
        slam,
        nav2,
        wombat,
    ])