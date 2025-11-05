from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic  = LaunchConfiguration('image_topic')
    green_low    = LaunchConfiguration('green_low')
    green_high   = LaunchConfiguration('green_high')
    red1_low     = LaunchConfiguration('red1_low')
    red1_high    = LaunchConfiguration('red1_high')
    red2_low     = LaunchConfiguration('red2_low')
    red2_high    = LaunchConfiguration('red2_high')
    kernel       = LaunchConfiguration('kernel')
    open_iters   = LaunchConfiguration('open_iters')
    close_iters  = LaunchConfiguration('close_iters')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic',  default_value='/camera/image'),
        DeclareLaunchArgument('green_low',    default_value='50,100,20'),
        DeclareLaunchArgument('green_high',   default_value='90,255,120'),
        DeclareLaunchArgument('red1_low',     default_value='0,150,50'),
        DeclareLaunchArgument('red1_high',    default_value='10,255,255'),
        DeclareLaunchArgument('red2_low',     default_value='170,150,50'),
        DeclareLaunchArgument('red2_high',    default_value='180,255,255'),
        DeclareLaunchArgument('kernel',       default_value='5'),
        DeclareLaunchArgument('open_iters',   default_value='1'),
        DeclareLaunchArgument('close_iters',  default_value='2'),

        Node(
            package='forestguard_colour',
            executable='tree_detector',
            name='tree_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'green_low': green_low,
                'green_high': green_high,
                'red1_low': red1_low,
                'red1_high': red1_high,
                'red2_low': red2_low,
                'red2_high': red2_high,
                'kernel': kernel,
                'open_iters': open_iters,
                'close_iters': close_iters,
            }]
        )
    ])
