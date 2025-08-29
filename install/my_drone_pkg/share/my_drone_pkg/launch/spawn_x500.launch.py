from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.join(os.environ['HOME'], 'ForestGuard', 'src', 'my_drone_pkg', 'models', 'x500')
    sdf_file = os.path.join(pkg_share, 'model.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', sdf_file],
            output='screen'
        )
    ])
