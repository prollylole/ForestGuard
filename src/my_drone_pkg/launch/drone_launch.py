# #!/usr/bin/env python3

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, SetEnvironmentVariable
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Get package share directory
#     pkg_share = get_package_share_directory('my_drone_pkg')
    
#     # Get ROS distribution
#     ros_distro = os.environ.get('ROS_DISTRO', 'humble')
#     ros_lib_path = f"/opt/ros/{ros_distro}/lib"
    
#     # Set the necessary environment variables
#     env_vars = [
#         # Set GAZEBO_PLUGIN_PATH to include ROS 2 plugins
#         SetEnvironmentVariable(
#             'GAZEBO_PLUGIN_PATH',
#             ros_lib_path + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')
#         ),
        
#         # Set model path
#         SetEnvironmentVariable(
#             'GAZEBO_MODEL_PATH',
#             os.path.join(pkg_share, 'models') + 
#             ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
#         ),
        
#         # Set resource path (for Ignition Gazebo)
#         SetEnvironmentVariable(
#             'GZ_SIM_RESOURCE_PATH',
#             os.path.join(pkg_share, 'models') + 
#             ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
#         ),
#     ]
    
#     # Gazebo process

#     gazebo_process = ExecuteProcess(
#         cmd=['gz', 'sim', '--verbose', os.path.join(pkg_share, 'worlds', 'world_demo.sdf')],
#         output='screen'
#     )
    
#     # Optional: Add ROS 2 nodes if needed
#     # For example, a node to interface with the drone
#     drone_node = Node(
#         package='my_drone_pkg',
#         executable='drone_controller',
#         output='screen'
#     )
    
#     return LaunchDescription(env_vars + [gazebo_process, drone_node])

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Locate the urdf file inside your package
    pkg_share = get_package_share_directory('my_drone_pkg')
    world_file = os.path.join(pkg_share, 'models', 'X4_GPS_LIDAR_RGBD', 'model.sdf')

    with open(world_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])