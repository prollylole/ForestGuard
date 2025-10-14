from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('forest_guard_sim')

    world = PathJoinSubstitution([pkg_share, 'worlds', 'forest.world.sdf'])
    models_dir = PathJoinSubstitution([pkg_share, 'models'])
    worlds_dir = PathJoinSubstitution([pkg_share, 'worlds'])

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', [models_dir, ':', worlds_dir]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [models_dir, ':', worlds_dir]),  # harmless if gz isn't installed
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            output='screen'
        ),
    ])