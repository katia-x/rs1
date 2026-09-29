"""Launch a complete Mission and Wombat integration test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


REGION = (
    "{header: {frame_id: map}, poses: ["
    "{position: {x: -2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}, "
    "{position: {x: 2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}, "
    "{position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}, "
    "{position: {x: -2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}]}"
)


def generate_launch_description():
    package_share = FindPackageShare('rs1')
    use_sim_time = LaunchConfiguration('use_sim_time')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package_share, 'launch', 'rs1_demo.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',
            'nav2': 'True',
            'husky': 'True',
            'parrot': 'False',
            'gui': 'True',
            'husky_x': '4.0',
            'husky_y': '4.0',
            'husky_z': '1.0',
            'husky_yaw': '0.0',
        }.items(),
    )

    mission_node = Node(
        package='rs1',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'wombat_base_frame': 'husky1_base_link',
        }],
        remappings=[
            ('/tf', '/husky1/tf'),
            ('/tf_static', '/husky1/tf_static'),
        ],
    )

    wombat_node = Node(
        package='rs1',
        executable='wombat_node',
        name='wombat_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'navigation_action': '/husky1/navigate_to_pose',
        }],
    )

    region_publisher = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/region',
                 'geometry_msgs/msg/PoseArray', REGION],
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        simulation,
        mission_node,
        wombat_node,
        region_publisher,
    ])