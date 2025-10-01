# -- Full simulation in Gazebo (gz_sim is included)
# -- ROS–Gazebo bridge (ros_gz_bridge) to convert topics/messages between ROS 2 and Gazebo
# -- Robot state publisher to publish transforms of robot links
# -- Optional RViz visualization
# -- Uses a config file for bridging topics (YAML)

# ** Launches Gazebo simulation with the diff_drive.sdf world.
# ** Bridges topics like /cmd_vel, /odom, etc. so that ROS 2 nodes can talk to Gazebo.
# ** Publishes TF for RViz visualization.
# ** Can run autonomous control or teleop because the bridge exposes control topics.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
import os
import xacro

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('my_drone_pkg')

    #  Path to urdf and sdf files inside package
    xacro_file = os.path.join(pkg_share, 'urdf', 'X4.urdf.xacro')
    world_file = os.path.join(pkg_share, 'models', 'X4_GPS_LIDAR_RGBD', 'model.sdf')

    # Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Load the SDF file from "description" package
    # with open(world_file, 'r') as infp:
    #     robot_desc = infp.read()
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': os.path.join(pkg_share, 'worlds', 'world_demo.sdf')}.items(),
    )

    # -- this is for robot control in rviz, can either use sdf through sdformat or urdf through xacro

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge_config.yaml')
        }],
        output='screen'
    )

    # Visualize in RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'drone.rviz')]
    )

    # -- if uses robot_state_publisher add it to the return list like this
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])