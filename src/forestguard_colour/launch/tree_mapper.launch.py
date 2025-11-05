from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic  = LaunchConfiguration('image_topic')
    scan_topic   = LaunchConfiguration('scan_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    base_frame   = LaunchConfiguration('base_frame')
    map_frame    = LaunchConfiguration('map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    trunk_r_min  = LaunchConfiguration('trunk_r_min')
    trunk_r_max  = LaunchConfiguration('trunk_r_max')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic',   default_value='/camera/image'),
        DeclareLaunchArgument('scan_topic',    default_value='/scan'),
        DeclareLaunchArgument('camera_frame',  default_value='camera_link'),
        DeclareLaunchArgument('base_frame',    default_value='base_link'),
        DeclareLaunchArgument('map_frame',     default_value='map'),
        DeclareLaunchArgument('use_sim_time',  default_value='true'),
        DeclareLaunchArgument('trunk_r_min',   default_value='0.06',
                              description='Minimum expected trunk radius (m)'),
        DeclareLaunchArgument('trunk_r_max',   default_value='0.50',
                              description='Maximum expected trunk radius (m)'),

        Node(
            package='forestguard_colour',
            executable='lidar_tree_mapper',
            name='lidar_tree_mapper',
            output='screen',
            parameters=[{
                'scan_topic':        scan_topic,
                'base_frame':        base_frame,
                'map_frame':         map_frame,
                'use_sim_time':      use_sim_time,

                # spacing model
                'min_sep_m':         3.0,     # world spacing
                'deadzone_frac':     0.5,     # deadzone = 0.5 * min_sep

                # clustering
                'cluster_break_m':   0.20,
                'adaptive_break_k':  2.0,

                # shape gates
                'trunk_r_min_m':     trunk_r_min,
                'trunk_r_max_m':     trunk_r_max,
                'max_range_m':       12.0,
                'min_arc_deg':       20.0,
                'max_cluster_span_m': 1.2,
                'circularity_min':    0.35,

                # markers
                'marker_scale_m':     0.35,
                'marker_lifetime_s':  0.0,
            }]
        ),

        Node(
            package='forestguard_colour',
            executable='camera_tree_mapper',
            name='camera_tree_mapper',
            output='screen',
            parameters=[{
                'image_topic':  image_topic,
                'scan_topic':   scan_topic,
                'camera_frame': camera_frame,
                'base_frame':   base_frame,
                'map_frame':    map_frame,
                'use_sim_time': use_sim_time,
            }]
        ),

        Node(
            package='forestguard_colour',
            executable='tree_colour_confirmer',
            name='tree_colour_confirmer',
            output='screen',
            parameters=[{
                'image_topic':      image_topic,
                'tree_pose_topic':  '/trees/poses',
                'map_frame':        map_frame,
                'base_frame':       base_frame,
                'use_sim_time':     use_sim_time,
            }]
        ),
    ])
