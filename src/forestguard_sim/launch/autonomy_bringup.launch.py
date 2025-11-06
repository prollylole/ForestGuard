from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_gate = DeclareLaunchArgument('use_lidar_gate', default_value='false')
    room_poly = DeclareLaunchArgument('room_polygon_xy', default_value='[]')   # string
    room_rad  = DeclareLaunchArgument('room_radius_m',  default_value='0.0')   # string ok

    lidar_gate = Node(
        package='forestguard_perception',
        executable='lidar_gate',
        name='lidar_gate',
        condition=IfCondition(LaunchConfiguration('use_lidar_gate')),
        parameters=[{
            'room_polygon_xy': LaunchConfiguration('room_polygon_xy'),
            'room_radius_m':   LaunchConfiguration('room_radius_m'),
        }],
        remappings=[('/scan', '/scan'), ('/scan_gated','/scan_gated')],
    )

    autonomy = Node(
        package='forestguard_controller',
        executable='autonomy_node',
        name='autonomy_behaviour',
        parameters=[{
            'settle_time_s': 10.0,
            'scan_time_s':   3.0,
            'confirm_range': 8.0,
            'batch_size':    4,
            'lidar_max_m':   12.0,
            'room_polygon_xy': LaunchConfiguration('room_polygon_xy'),  # string, parsed in node
            'room_radius_m':   LaunchConfiguration('room_radius_m'),
            'min_separation_m': 2.0,
            'furthest_bias': True,
            'joy_rb_index': 5,
        }],
    )

    return LaunchDescription([use_gate, room_poly, room_rad, lidar_gate, autonomy])
