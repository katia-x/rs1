
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def generate_launch_description():
    package_path = FindPackageShare('rs1')
    config_path = PathJoinSubstitution([package_path, 'config'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless_rendering = LaunchConfiguration('headless_rendering')

    server_config_file = PathJoinSubstitution([
        config_path,
        'ignition_server.config',
    ])

    wombat_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                package_path,
                'urdf_husky',
                'husky.urdf.xacro',
            ]),
            ' prefix:=husky1_ gz_model_name:=husky1',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Open the Gazebo GUI',
        ),
        DeclareLaunchArgument(
            'headless_rendering',
            default_value='False',
            description='Use EGL for camera rendering without a display',
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[package_path, ':', EnvironmentVariable(
                'IGN_GAZEBO_RESOURCE_PATH', default_value='')],
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[package_path, ':', EnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', default_value='')],
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SERVER_CONFIG_PATH',
            value=server_config_file,
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SERVER_CONFIG_PATH',
            value=server_config_file,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py',
            ])),
            launch_arguments={
                'ign_args': [
                    PathJoinSubstitution([
                        package_path,
                        'worlds',
                        'rs1_terrain.config',
                    ]),
                    ' -r',
                    PythonExpression(["' ' if "] + _is_true_expression('gui') + [
                        " else ' -s'",
                    ]),
                    PythonExpression(["' --headless-rendering' if "] +
                                     _is_true_expression('headless_rendering') + [
                                         " else ''",
                                     ]),
                ],
                'on_exit_shutdown': 'True',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='husky1',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': wombat_description,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[Node(
                package='ros_ign_gazebo',
                executable='create',
                name='spawn_wombat',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-topic',
                    '/husky1/robot_description',
                    '-name',
                    'husky1',
                    '-x',
                    '-0.5',
                    '-y',
                    '0.0',
                    '-z',
                    '0.5',
                    '-Y',
                    '0.0',
                ],
            )],
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_clock',
            output='screen',
            parameters=[{
                'config_file': PathJoinSubstitution([
                    config_path,
                    'gazebo_bridge_clock.yaml',
                ]),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])