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
    # CHANGED: use your 'john' package instead of 'forest_guard_sim'
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

    # CHANGED: default map path now points inside your john package
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([john_pkg, 'config', 'forest_map.yaml']),
        description='Full path to map yaml file to load'
    )

    # CHANGED: default params path now points inside your john package
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([john_pkg, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Map server node from nav2_params.yaml
    # map_server_cmd = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': map_yaml_file
    #     }]
    # )

    # AMCL node for localization
    # amcl_cmd = Node(
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': PathJoinSubstitution([john_pkg, 'config', 'amcl_params.yaml'])
    #     }]
    # )

    # Lifecycle manager - FIXED: Proper node names
    # lifecycle_manager_cmd = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_navigation',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': True,
    #         'node_names': [
    #             'map_server',
    #             'amcl',
    #             'controller_server',
    #             'local_costmap',
    #             'global_costmap',
    #             'planner_server',
    #             'smoother_server',
    #             'behavior_server',
    #             'bt_navigator',
    #             'waypoint_follower',
    #             'velocity_smoother'
    #         ]
    #     }]
    # )

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

    # Group with remapping for odometry
    nav2_bringup_cmd = GroupAction([
        SetRemap(src='/odom', dst='/odometry/filtered'),
        nav2_bringup_include
    ])

    # Add all commands to launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    # ld.add_action(map_server_cmd)
    # ld.add_action(amcl_cmd)
    # ld.add_action(lifecycle_manager_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
