"""RS1 two-robot simulation on ROS 2 Humble / Gazebo Fortress."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    # ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, RosTimer
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"

# MISSION_REGION = (
#     "{header: {frame_id: map}, poses: ["
#     "{position: {x: -5.0, y: -5.0, z: 0.0}, orientation: {w: 1.0}}, "
#     "{position: {x: 5.0, y: -5.0, z: 0.0}, orientation: {w: 1.0}}, "
#     "{position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}, "
#     "{position: {x: -5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}]}"
# )

def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _if_all_true(*names):
    pieces = []
    for i, name in enumerate(names):
        if i > 0:
            pieces.append(' and ')
        pieces.extend(_is_true_expression(name))
    return IfCondition(PythonExpression(pieces))


def add_robot(
    ld,
    *,
    pkg_path,
    config_path,
    world,
    use_sim_time,
    enabled_arg,
    namespace,
    frame_prefix,
    gz_model_name,
    xacro_parts,
    bridge_config,
    localization_config,
    x,
    y,
    z,
    yaw='0.0',
    spawn_delay=0.0,
):
    """Add one robot instance to the launch description.

    Convention used here:
      - ROS nodes/topics are under /<namespace>/...
      - Gazebo entity/model is named <gz_model_name>
      - Gazebo plugin topics are under /model/<gz_model_name>/...
      - TF topics are under /<namespace>/tf and /<namespace>/tf_static
      - TF frame ids use <frame_prefix>, e.g. husky1_base_link
    """

    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path] + xacro_parts),
            ' ',
            'prefix:=', frame_prefix,
            ' ',
            'gz_model_name:=', gz_model_name,
        ]),
        value_type=str,
    )

    robot_enabled = _if_true(enabled_arg)

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, localization_config]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
        ],
        condition=robot_enabled,
    ))

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_' + namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-world', world,
            '-topic', '/' + namespace + '/robot_description',
            '-name', gz_model_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )

    # Start Gazebo first, then insert robot entities in a predictable order.
    ld.add_action(RosTimer(
        period=spawn_delay,
        actions=[robot_spawner],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, bridge_config]),
            'use_sim_time': use_sim_time,
        }],
        condition=robot_enabled,
    ))


def add_navigation_instance(
    ld,
    *,
    pkg_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    start_delay,
):
    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_path,
                'launch',
                'rs1_navigation.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': robot_namespace,
            'config_filename_suffix': '_' + robot_namespace,
            'slam': LaunchConfiguration('slam'),
            'nav2': LaunchConfiguration('nav2'),
        }.items(),
    )

    # Scope is important: without it, repeated IncludeLaunchDescription calls can
    # leak/overwrite launch configurations such as namespace and config suffix.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[GroupAction(
            scoped=True,
            actions=[nav_include],
        )],
        condition=_if_true(robot_arg),
    ))


def add_rviz_instance(
    ld,
    *,
    config_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    rviz_config,
    start_delay,
):
    # Each RViz process is itself placed in the robot namespace so the Nav2 RViz
    # panel resolves relative action/service names such as navigate_to_pose and
    # lifecycle_manager_navigation to the correct robot.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_namespace,
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([config_path, rviz_config])],
            remappings=[
                ('/tf', '/' + robot_namespace + '/tf'),
                ('/tf_static', '/' + robot_namespace + '/tf_static'),
            ],
        )],
        condition=_if_all_true('rviz', robot_arg),
    ))


def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('rs1')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # C++ OpenCV inference; no training environment required at runtime.
    ld.add_action(DeclareLaunchArgument('perception', default_value='True',
        description='Run the C++ plant detector for Husky'))
    ld.add_action(DeclareLaunchArgument('device', default_value='auto',
        choices=['auto', 'cpu', 'cuda'], description='OpenCV inference backend'))
    ld.add_action(DeclareLaunchArgument('model_path', default_value=PathJoinSubstitution([
        pkg_path, 'machine_learning', 'models', 'rtmdet_tiny_plants_v2.onnx'])))
    ld.add_action(DeclareLaunchArgument('camera_topic', default_value='/husky1/camera/image'))
    ld.add_action(DeclareLaunchArgument('camera_reliability', default_value='reliable',
        choices=['best_effort', 'reliable']))
    ld.add_action(DeclareLaunchArgument('confidence_threshold', default_value='0.3'))
    ld.add_action(DeclareLaunchArgument('nms_threshold', default_value='0.65'))
    ld.add_action(DeclareLaunchArgument('inference_rate_hz', default_value='2.0'))


    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz',
    )
    ld.add_action(rviz_launch_arg)

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Flag to launch SLAM Toolbox for each enabled robot',
    )
    ld.add_action(slam_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2 for each enabled robot. Nav2 also starts SLAM.',
    )
    ld.add_action(nav2_launch_arg)

    ld.add_action(DeclareLaunchArgument(
        'mission_region_delay', default_value='20.0',
        description='Simulation seconds before publishing the mission search region'))

    husky_launch_arg = DeclareLaunchArgument(
        'husky',
        default_value='True',
        description='Launch the Husky UGV instance in namespace /husky1',
    )
    ld.add_action(husky_launch_arg)

    parrot_launch_arg = DeclareLaunchArgument(
        'parrot',
        default_value='True',
        description='Launch the Parrot drone instance in namespace /parrot1',
    )
    ld.add_action(parrot_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='rs1_terrain',
        description='Which world to load',
    )
    ld.add_action(world_launch_arg)

    world_file_launch_arg = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description='World filename under worlds/, including its extension',
    )
    ld.add_action(world_file_launch_arg)

    ld.add_action(DeclareLaunchArgument(
        'gui', default_value='True', description='Open the Gazebo GUI'))
    ld.add_action(DeclareLaunchArgument(
        'headless_rendering', default_value='False',
        description='Use EGL for camera rendering without a display'))
    for robot, defaults in [('husky', ('-0.5', '0.0', '0.5', '0.0')),
                            ('parrot', ('2.0', '0.0', '1.5', '0.0'))]:
        for axis, value in zip(('x', 'y', 'z', 'yaw'), defaults):
            ld.add_action(DeclareLaunchArgument(
                robot + '_' + axis, default_value=value,
                description=robot + ' spawn ' + axis))

    # The share directory contains worlds/ and its model meshes.
    for variable in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        ld.add_action(SetEnvironmentVariable(
            name=variable,
            value=[pkg_path, ':', EnvironmentVariable(variable, default_value='')]))

    # Load common Gazebo server systems from a shared config file. This keeps
    # required systems such as Sensors out of individual robot and world files,
    # so custom student worlds do not need to copy plugin blocks.
    server_config_file = PathJoinSubstitution([
        config_path,
        'ignition_server.config',
    ])
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))

    # Start Gazebo once.
    world_file = LaunchConfiguration('world_file')
    world_path = PathJoinSubstitution([
        pkg_path,
        'worlds',
        PythonExpression([
            "'", world_file, "' if '", world_file, "' else (",
            "'rs1_terrain.config' if '", LaunchConfiguration('world'),
            "' == 'rs1_terrain' else '", LaunchConfiguration('world'), ".sdf')",
        ]),
    ])
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'ign_args': [
                world_path,
                ' -r',
                PythonExpression(["' ' if "] + _is_true_expression('gui') + [" else ' -s'"]),
                PythonExpression(["' --headless-rendering' if "] +
                                 _is_true_expression('headless_rendering') + [" else ''"]),
            ],
            'on_exit_shutdown': 'True',
        }.items(),
    ))

    # Bridge the global simulation clock once.
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        world=LaunchConfiguration('world'),
        use_sim_time=use_sim_time,
        enabled_arg='husky',
        namespace='husky1',
        frame_prefix='husky1_',
        gz_model_name='husky1',
        xacro_parts=['urdf_husky', 'husky.urdf.xacro'],
        bridge_config='gazebo_bridge_husky1.yaml',
        localization_config='robot_localization_husky1.yaml',
        x=LaunchConfiguration('husky_x'),
        y=LaunchConfiguration('husky_y'),
        z=LaunchConfiguration('husky_z'),
        yaw=LaunchConfiguration('husky_yaw'),
        spawn_delay=3.0,
    )

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        world=LaunchConfiguration('world'),
        use_sim_time=use_sim_time,
        enabled_arg='parrot',
        namespace='parrot1',
        frame_prefix='parrot1_',
        gz_model_name='parrot1',
        xacro_parts=['urdf_parrot', 'parrot.urdf.xacro'],
        bridge_config='gazebo_bridge_parrot1.yaml',
        localization_config='robot_localization_parrot1.yaml',
        x=LaunchConfiguration('parrot_x'),
        y=LaunchConfiguration('parrot_y'),
        z=LaunchConfiguration('parrot_z'),
        yaw=LaunchConfiguration('parrot_yaw'),
        spawn_delay=6.0,
    )

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        start_delay=15.0,
    )

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        start_delay=10.0,
    )

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        rviz_config='rs1_husky1.rviz',
        start_delay=11.0,
    )

    # add_rviz_instance(
    #     ld,
    #     config_path=config_path,
    #     use_sim_time=use_sim_time,
    #     robot_arg='parrot',
    #     robot_namespace='parrot1',
    #     rviz_config='rs1_parrot1.rviz',
    #     start_delay=13.0,
    # )

    ld.add_action(Node(
        package='rs1', executable='plant_detector', name='plant_detector',
        namespace='husky1', output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_reliability': LaunchConfiguration('camera_reliability'),
            'confidence_threshold': ParameterValue(LaunchConfiguration('confidence_threshold'), value_type=float),
            'nms_threshold': ParameterValue(LaunchConfiguration('nms_threshold'), value_type=float),
            'inference_rate_hz': ParameterValue(LaunchConfiguration('inference_rate_hz'), value_type=float),
            'use_sim_time': use_sim_time,
        }],
        condition=_if_all_true('perception', 'husky'),
    ))

    ld.add_action(Node(
        package='rs1', executable='mission_node', name='mission_node',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'mission_params.yaml']), {
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'wombat_base_frame': 'husky1_base_link',
        }],
        remappings=[
            ('/tf', '/husky1/tf'),
            ('/tf_static', '/husky1/tf_static'),
        ],
        condition=_if_true('husky'),
    ))

    ld.add_action(Node(
        package='rs1', executable='wombat_node', name='wombat_node',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'mission_params.yaml']), {
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'navigation_action': '/husky1/navigate_to_pose',
        }],
        condition=_if_true('husky'),
    ))

    # ld.add_action(TimerAction(
    #     period=LaunchConfiguration('mission_region_delay'),
    #     actions=[ExecuteProcess(
    #         cmd=[
    #             'ros2', 'topic', 'pub', '--once',
    #             '/region', 'geometry_msgs/msg/PoseArray', MISSION_REGION,
    #         ],
    #         output='screen',
    #     )],
    #     condition=_if_true('husky'),
    # ))

    return ld
