from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    scan_topic  = LaunchConfiguration('scan_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('scan_topic',  default_value='/scan'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('base_frame',   default_value='base_link'),
        DeclareLaunchArgument('map_frame',    default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='forestguard_colour',
            executable='tree_mapper',
            name='tree_mapper',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'scan_topic':  scan_topic,
                'camera_frame': camera_frame,
                'base_frame':   base_frame,
                'map_frame':    map_frame,
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
