from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forestguard_behaviour',
            executable='autonomy',
            name='autonomy_behaviour',
            output='screen',
            parameters=[{
                'settle_time_s': 5.0,
                'scan_time_s': 3.0,
                'confirm_range': 8.0,
                'batch_size': 4,
                'lidar_max_m': 12.0,
                'room_radius_m': 50.0,
                'room_polygon_xy': '[]',
                'min_separation_m': 2.0,
                'furthest_bias': True,
                'perception_trigger_service': '/perception/snapshot',
                'start_topic': '/ui/start_mission',
                'amcl_topic': '/amcl_pose',
            }]
        )
    ])
