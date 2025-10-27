from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    ld = LaunchDescription()

    # Get package paths
    john_pkg = FindPackageShare('john')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Default map path inside your john package
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([john_pkg, 'config', 'forest_map.yaml']),
        description='Full path to map yaml file to load'
    )

    # Default params path inside your john package
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([john_pkg, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Include Nav2 bringup for the rest of navigation stack
    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml_file,
            'autostart': 'true',
            'map_subscribe_transient_local': 'true'
        }.items(),
    )

    # Group with remapping for odometry (Nav2 uses /odom; we feed EKF's /odometry/filtered)
    nav2_bringup_cmd = GroupAction([
        SetRemap(src='/odom', dst='/odometry/filtered'),
        nav2_bringup_include
    ])

    # Add all commands to launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
