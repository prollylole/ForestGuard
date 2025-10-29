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
            executable='camera_tree_mapper',
            name='camera_tree_mapper',
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

        Node(
            package='forestguard_colour',
            executable='lidar_tree_mapper',
            name='lidar_tree_mapper',
            output='screen',
            parameters=[{
                'scan_topic':  LaunchConfiguration('scan_topic'),   # default /scan
                'base_frame':  LaunchConfiguration('base_frame'),   # default base_link
                'map_frame':   LaunchConfiguration('map_frame'),    # default map
                'deadzone_m':  0.8,    # tune with your spawn spacing
                'cluster_break_m': 0.20,
            }]
        ),
        
        # Node(
        #     package='forestguard_colour',
        #     executable='tree_colour_confirmer',
        #     name='tree_colour_confirmer',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': '/camera/image',
        #         'map_frame':   'map',
        #         'base_frame':  'base_link',
        #         'camera_hfov_deg': 120,
        #         'roi_ymin_frac': 0.35,
        #         'stripe_half_px': 16,
        #         'min_votes': 40,
        #     }]
        # )
    ])
