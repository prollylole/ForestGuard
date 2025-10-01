#!/usr/bin/env python3
# """
# Combined launch: start Ignition world (forest) and spawn the X4 drone from my_drone_pkg.

# Usage example:
#   ros2 launch <your_pkg> forest_with_drone.launch.py world:=forest_random rviz:=True

# Notes:
#  - Ensure forest_guard_sim/worlds/<world>.sdf exists (default 'forest_random' here).
#  - Ensure my_drone_pkg contains urdf/X4.urdf.xacro
#  - Ensure both packages' models/ and worlds/ folders exist and contain models/worlds referred by SDFs.
# """

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
                              
def generate_launch_description():
    ld = LaunchDescription()

    # Launch args
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='forest_random',  # change if your packaged world has a different name
        description='Which world file (SDF name without .sdf) to load from forest_guard_sim/worlds'
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='False',
                                     description='Start RViz?')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True',
                                             description='Use sim time')
    ld.add_action(world_arg)
    ld.add_action(rviz_arg)
    ld.add_action(use_sim_time_arg)

    # Packages
    forest_pkg = FindPackageShare('forest_guard_sim')
    drone_pkg = FindPackageShare('my_drone_pkg')
    ign_launch_pkg = FindPackageShare('ros_ign_gazebo')  # sample uses ros_ign_gazebo's ign_gazebo.launch.py

    # --- Ensure Ignition can find both package models/worlds ---
    # Build a PATH string that includes forest_pkg/models and forest_pkg/worlds,
    # and drone_pkg/models and drone_pkg/worlds (if any).
    # We use PathJoinSubstitution so substitutions are resolved at launch time.
    forest_models = PathJoinSubstitution([forest_pkg, 'models'])
    forest_worlds = PathJoinSubstitution([forest_pkg, 'worlds'])
    drone_models = PathJoinSubstitution([drone_pkg, 'models'])
    drone_worlds = PathJoinSubstitution([drone_pkg, 'worlds'])

    # Set the environment variables used by Ignition to locate model:// URIs
    # Note: using PathJoinSubstitution here ensures we don't hardcode absolute paths.
    # We set both IGN_GAZEBO_RESOURCE_PATH and GZ_SIM_RESOURCE_PATH for compatibility.
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[forest_models, os.pathsep, forest_worlds, os.pathsep, drone_models, os.pathsep, drone_worlds]
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[forest_models, os.pathsep, forest_worlds, os.pathsep, drone_models, os.pathsep, drone_worlds]
    ))

    # --- Robot description (xacro -> robot_description) ---
    # Command substitution runs xacro and embeds the URDF string
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([drone_pkg, 'urdf', 'X4.urdf.xacro'])]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    ld.add_action(robot_state_publisher_node)

    # --- Start Ignition Gazebo (load chosen world) ---
    # This uses the ros_ign_gazebo launch (sample). If you use ros_gz_sim, swap to its gz_sim.launch.py
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ign_launch_pkg, 'launch', 'ign_gazebo.launch.py'])
        ),
        launch_arguments={
            # ign_args expects the world path and -r to run
            'ign_args': [PathJoinSubstitution([forest_pkg, 'worlds',
                                             [LaunchConfiguration('world'), '.sdf']]), ' -r']
        }.items()
    )
    ld.add_action(ign_gazebo)

    # --- Spawn the robot into the running Ignition world ---
    # The 'ros_ign_gazebo create' node will take robot description from /robot_description
    # and spawn it. We pass -z to set initial height (change as needed).
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_robot_from_urdf',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-topic', '/robot_description', '-z', '0.5']  # adjust z if needed
    )
    ld.add_action(robot_spawner)

    # --- Bridge (ros_ign_bridge / ros_gz_bridge) ---
    # If you have a YAML bridge config in forest_guard_sim/config, point to it.
    # Otherwise the bridge can be set up as you had it before (ros_gz_bridge parameter_bridge).
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_ign_bridge',
        output='screen',
        parameters=[{
            # Update path below if your bridge config is located elsewhere
            'config_file': PathJoinSubstitution([forest_pkg, 'config', 'gazebo_bridge.yaml']),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    ld.add_action(bridge)

    # --- Optional RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([forest_pkg, 'config', 'forest.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    ld.add_action(rviz_node)

    # (Optional) add nav2 include like the sample if you have navigation files in forest_pkg
    # nav2 = IncludeLaunchDescription(...)

    return ld




















# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # names of the existing packages that own the forest and drone
    forest_pkg = get_package_share_directory('forest_guard_sim')   
    drone_pkg  = get_package_share_directory('my_drone_pkg')       

    # spawn droen at specified (x,y,z) or default (0,0,0.5)
    drone_x = LaunchConfiguration('drone_x', default='0.0')
    drone_y = LaunchConfiguration('drone_y', default='0.0')
    drone_z = LaunchConfiguration('drone_z', default='0.5')

    # --- Robustly append model paths to IGN_GAZEBO_RESOURCE_PATH and GZ_SIM_RESOURCE_PATH ---
    forest_models = os.path.join(forest_pkg, 'models')
    drone_models  = os.path.join(drone_pkg, 'models')

    def append_paths(env_var, paths_to_add):
        existing = os.environ.get(env_var, '').split(os.pathsep)
        # remove empty strings and duplicates
        existing = [p for p in existing if p]
        for p in paths_to_add:
            if p not in existing:
                existing.append(p)
        return os.pathsep.join(existing)

    ign_path = append_paths('IGN_GAZEBO_RESOURCE_PATH', [forest_models, drone_models])
    gz_path  = append_paths('GZ_SIM_RESOURCE_PATH', [forest_models, drone_models])

    return LaunchDescription([
        # make models/world visible to ign (include both package model folders if needed)
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_path),

        # allow user to override drone spawn position on CLI
        DeclareLaunchArgument('drone_x', default_value='0.0'),
        DeclareLaunchArgument('drone_y', default_value='0.0'),
        DeclareLaunchArgument('drone_z', default_value='0.5'),

        # Launch the forest world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(forest_pkg, 'launch', 'forest_random.launch.py')
            )
        ),

        # Launch the drone spawn/control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_pkg, 'launch', 'drone_launch.py')
            ),
            launch_arguments={'x': drone_x, 'y': drone_y, 'z': drone_z}.items()
        ),
    ])
