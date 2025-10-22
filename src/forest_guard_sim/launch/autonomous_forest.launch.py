from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz')
    use_map = LaunchConfiguration('use_map', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )

    declare_use_map_cmd = DeclareLaunchArgument(
        'use_map',
        default_value='true',
        description='Use prebuilt map for navigation'
    )

    # Include your forest simulation
    forest_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('forest_guard_sim'),
                'launch',
                'large_forest_husky.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'true',  # We'll launch our own RViz
            'nav2': 'false'   # We're using our custom navigation
        }.items()
    )

    # # Husky controller node
    # husky_controller_cmd = Node(
    #     package='husky_controller',
    #     executable='husky',
    #     name='husky_controller',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Add static transform from map to odom (FIXES MAP FRAME ISSUE)
    static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Navigation with prebuilt map
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('forest_guard_sim'),
                'launch',
                'navigation_map.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_map)
    )

    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
                FindPackageShare('forest_guard_sim'),
                'config',
                'forest.rviz'
    ])
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
    )

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_map_cmd)
    ld.add_action(forest_sim_cmd)
    # ld.add_action(husky_controller_cmd)  # Add your controller
    ld.add_action(static_transform_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld