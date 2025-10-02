from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('john')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Forest world package
    forest_pkg = get_package_share_directory('john')
    forest_world = PathJoinSubstitution([forest_pkg, 'worlds', 'forest.world.sdf'])
    forest_models = PathJoinSubstitution([forest_pkg, 'models'])
    forest_worlds = PathJoinSubstitution([forest_pkg, 'worlds'])

    # Launch args
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(nav2_launch_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_state_publisher_node)

    # Localization
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Gazebo (forest world only)
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', [forest_models, ':', forest_worlds]))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [forest_models, ':', forest_worlds]))
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', forest_world],
        output='screen'
    )
    ld.add_action(gazebo)

    # Spawn robot
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # Gazebo bridge
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, 'john.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', 'johnNavigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
