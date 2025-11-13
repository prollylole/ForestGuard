from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os
import math


def _resolve_spawn_arg(context, name, lo, hi):
    # Currently just returns the value; you can clamp to [lo, hi] if you want later
    val = LaunchConfiguration(name).perform(context)
    return val


def _setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('forestguard_sim')
    world_path = os.path.join(pkg_path, 'worlds', 'forest_trees.sdf')

    spawn_x = _resolve_spawn_arg(context, 'spawn_x', -2.0, 2.0)
    spawn_y = _resolve_spawn_arg(context, 'spawn_y', -2.0, 2.0)
    spawn_yaw = _resolve_spawn_arg(context, 'spawn_yaw', 0.0, 2 * math.pi)

    gz = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen',
    )
    on_close = RegisterEventHandler(
        OnProcessExit(target_action=gz, on_exit=[Shutdown(reason='Ignition GUI closed')])
    )

    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-topic', '/robot_description',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.0',
            '-Y', spawn_yaw,
            '-wait', '5'
        ]
    )
    return [gz, on_close, spawner]


def generate_launch_description():
    pkg_path = get_package_share_directory('forestguard_sim')
    colour_pkg_path = get_package_share_directory('forestguard_perception')

    resource_path = os.pathsep.join([
        os.path.join(pkg_path, 'models'),
        os.path.join(pkg_path, 'worlds'),
    ])

    autonomy_params_path = PathJoinSubstitution([pkg_path, 'config', 'autonomy_params.yaml'])
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    ld = LaunchDescription()

    # Environment vars for Ignition/Gazebo resources
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Common launch args
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))
    ld.add_action(DeclareLaunchArgument('ui', default_value='True'))
    ld.add_action(DeclareLaunchArgument('teleop', default_value='True'))
    ld.add_action(DeclareLaunchArgument('autonomy', default_value='True'))

    ld.add_action(DeclareLaunchArgument('spawn_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value='0.0'))

    # Robot description (xacro)
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # EKF (robot_localization)
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    ))

    # Gazebo ↔ ROS bridge
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge.yaml']),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # UI + teleop stack
    ld.add_action(Node(
        package='forestguard_ui', executable='run_ui', output='screen',
        condition=IfCondition(LaunchConfiguration('ui'))
    ))
    ld.add_action(Node(
        package='forestguard_ui', executable='controller_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'xbox_teleop.yaml'])],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='forestguard_ui',
        executable='twist_scaler',
        name='twist_scaler',
        parameters=[{'in_topic': '/cmd_vel_raw', 'out_topic': '/cmd_vel'}]
    ))

    # Perception: LiDAR tree mapper
    ld.add_action(Node(
        package='forestguard_perception',
        executable='lidar_tree_mapper',
        name='lidar_tree_mapper',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'min_sep_m': 3.0,
            'deadzone_frac': 0.5,
            'cluster_break_m': 0.20,
            'adaptive_break_k': 2.0,
            'trunk_r_min_m': 0.06,
            'trunk_r_max_m': 0.50,
            'max_range_m': 12.0,
            'min_arc_deg': 20.0,
            'max_cluster_span_m': 1.2,
            'circularity_min': 0.35,
            'marker_scale_m': 0.35,
            'marker_lifetime_s': 0.0,
        }]
    ))

    # Perception: camera + colour confirmation
    ld.add_action(Node(
        package='forestguard_perception',
        executable='camera_tree_mapper',
        name='camera_tree_mapper',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'scan_topic': '/scan',
            'camera_frame': 'camera_link',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    ))

    ld.add_action(Node(
        package='forestguard_perception',
        executable='tree_colour_confirmer',
        name='tree_colour_confirmer',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'tree_pose_topic': '/trees/poses',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    ))

    # Controller + autonomy behaviour
    ld.add_action(Node(
        package='forestguard_controller',
        executable='forestguard_controller',
        name='forestguard_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    ))

    ld.add_action(Node(
        package='forestguard_behaviour',
        executable='autonomy',
        name='autonomy_behaviour',
        output='screen',
        parameters=[
            autonomy_params_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('autonomy'))
    ))

    # Battery simulator
    ld.add_action(Node(
        package='forestguard_ui',
        executable='battery_sim',
        name='battery_sim',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'battery_topic': '/battery_state',
            'cmd_topic': '/cmd_vel',
            # tuneables (optional):
            # 'idle_drain_per_sec': 1.0/600.0,
            # 'k_lin': 3.0,
            # 'k_ang': 1.5,
            # 'v_max': 0.6,
            # 'w_max': 1.5,
            # 'voltage_full': 12.6,
            # 'voltage_empty': 9.6,
        }]
    ))

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'forest.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    ld.add_action(rviz_node)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=rviz_node, on_exit=[Shutdown(reason='RViz closed')])
    ))

    # SLAM toolbox (online_async) – from your second snippet
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(slam)

    # Nav2 navigation_launch – from your second snippet
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(navigation)

    # Bring up Ignition + robot spawner last
    ld.add_action(OpaqueFunction(function=_setup))

    return ld
