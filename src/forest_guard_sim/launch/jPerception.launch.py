from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # -----------------------------------------------------------
    # Get paths to directories
    # -----------------------------------------------------------
    pkg_path = FindPackageShare('john')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # -----------------------------------------------------------
    # Launch Arguments
    # -----------------------------------------------------------
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    ld.add_action(use_sim_time_launch_arg)
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2 navigation stack'
    )
    ld.add_action(nav2_launch_arg)

    teleop_launch_arg = DeclareLaunchArgument(
        'teleop',
        default_value='True',
        description='Flag to enable Xbox teleoperation'
    )
    ld.add_action(teleop_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='forest_world_green',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'forest_world', 'forest_world_green']
    )
    ld.add_action(world_launch_arg)

    # -----------------------------------------------------------
    # Robot Description & Localization
    # -----------------------------------------------------------
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # -----------------------------------------------------------
    # Joystick Teleoperation
    # -----------------------------------------------------------
    teleop_joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        condition=IfCondition(LaunchConfiguration('teleop'))
    )
    ld.add_action(teleop_joy_node)

    teleop_twist_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[PathJoinSubstitution([config_path, 'xbox_teleop.yaml'])],
        remappings=[('/cmd_vel', '/cmd_vel')],
        condition=IfCondition(LaunchConfiguration('teleop'))
    )
    ld.add_action(teleop_twist_node)

    # -----------------------------------------------------------
    # Gazebo Simulation
    # -----------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([
                pkg_path, 'worlds',
                [LaunchConfiguration('world'), '.sdf']
            ]), ' -r']
        }.items()
    )
    ld.add_action(gazebo)

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # -----------------------------------------------------------
    # Bridges & Visualisation
    # -----------------------------------------------------------
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(gazebo_bridge)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, 'john.rviz'])],
        env={'QT_QPA_PLATFORM': 'xcb'},  # fixes Wayland bug
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # -----------------------------------------------------------
    # Navigation Stack (Nav2)
    # -----------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', 'johnNavigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # -----------------------------------------------------------
    # Perception: Colour-based Tree Detector (forestguard_colour)
    # -----------------------------------------------------------
    perception_node = Node(
        package='forestguard_colour',
        executable='tree_detector_v2',
        name='tree_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'green_low': '35,60,40',
            'green_high': '85,255,255',
            'red1_low': '0,70,50',
            'red1_high': '10,255,255',
            'red2_low': '170,70,50',
            'red2_high': '180,255,255',
            'kernel': 5,
            'open_iters': 1,
            'close_iters': 2,
            'roi_ymin': 0.25,
            'min_area_px': 1200,
            'aspect_min': 1.2
        }]
    )
    ld.add_action(perception_node)

    return ld
