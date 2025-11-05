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
from ament_index_python.packages import get_package_share_directory

import os, math

# ------------------------
# Helpers
# ------------------------
def _resolve_spawn_arg(context, name, lo, hi):
    val = LaunchConfiguration(name).perform(context)
    return val

# ------------------------
# Setup: start Gazebo with existing world, spawn robot
# ------------------------
def _setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('john')
    world_path = os.path.join(pkg_path, 'worlds', 'forest_trees.sdf')

    # Spawn pose (keep your behavior: fixed values or RANDOM)
    spawn_x = _resolve_spawn_arg(context, 'spawn_x', -2.0, 2.0)
    spawn_y = _resolve_spawn_arg(context, 'spawn_y', -2.0, 2.0)
    spawn_yaw = _resolve_spawn_arg(context, 'spawn_yaw', 0.0, 2*math.pi)

    # Start Ignition with the existing world (no writing)
    gz = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen',
    )
    on_close = RegisterEventHandler(
        OnProcessExit(target_action=gz, on_exit=[Shutdown(reason='Ignition GUI closed')])
    )

    # Spawn Husky from /robot_description
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-topic', '/robot_description',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.2',
            '-Y', spawn_yaw,
            '-wait', '5'
        ]
    )
    return [gz, on_close, spawner]

# ------------------------
# Main launch description
# ------------------------
def generate_launch_description():
    pkg_path = get_package_share_directory('john')
    resource_path = os.pathsep.join([
        os.path.join(pkg_path, 'models'),
        os.path.join(pkg_path, 'worlds'),
    ])

    ld = LaunchDescription()

    # Gazebo model/resource paths
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Common args
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))
    ld.add_action(DeclareLaunchArgument('ui', default_value='True'))
    ld.add_action(DeclareLaunchArgument('teleop', default_value='True'))

    # Spawn pose (keep RANDOM support if you want it)
    ld.add_action(DeclareLaunchArgument('spawn_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value='0.0'))

    # Robot description
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

    # EKF
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    ))

    # Bridge (uses your existing YAML)
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge.yaml']),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    ))

    # UI + Teleop
    ld.add_action(Node(
        package='forestguard_ui', executable='run_ui', output='screen',
        condition=IfCondition(LaunchConfiguration('ui'))
    ))
    ld.add_action(Node(
        package='forestguard_ui', executable='controller_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'xbox_teleop.yaml'])],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='forestguard_ui', executable='twist_scaler', name='twist_scaler',
        parameters=[{'in_topic': '/cmd_vel_raw', 'out_topic': '/cmd_vel'}]
    ))

    # RViz (optional)
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

    # Nav2 (optional)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_path, 'launch', 'johnNavigation.launch.py'])]),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    ))

    # Start Gazebo with forest_trees.sdf + spawn robot
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
