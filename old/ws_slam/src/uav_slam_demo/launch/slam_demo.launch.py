from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    world_path = PathJoinSubstitution([FindPackageShare('uav_slam_demo'), 'worlds', 'uav_rgbd_world.sdf'])

    # Gazebo (gz sim) with our world
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Bridge the necessary topics between gz and ROS 2
    bridge_args = [
        # clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # IMU
        '/world/default/model/uav/link/base_link/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # Camera RGB, depth, info
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        # Point cloud (for visualisation)
        '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        # Cmd vel from ROS → gz sim velocity control
        '/model/uav/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=bridge_args
    )

    # IMU filter to stabilise orientation and produce /imu/filtered in ENU
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'use_mag': False},
            {'world_frame': 'enu'},
            {'publish_tf': False},
            {'use_sim_time': True},
        ],
        remappings=[
            ('/imu/data_raw', '/world/default/model/uav/link/base_link/imu'),
            ('/imu/data', '/imu/filtered'),
        ]
    )

    # Static TF base_link -> camera_link (matches SDF pose)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.10', '0', '0.05', '0', '0', '0', 'base_link', 'camera_link']
    )

    # RGBD Odometry (produces /odom)
    rgbd_odom = Node(
        package='rtabmap_ros',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
            {'frame_id': 'base_link'},
            {'wait_for_transform': 0.2},
            {'approx_sync': True},
            {'publish_tf': False},
            {'Odom/Strategy': '1'},
            {'use_sim_time': True},
        ],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('depth/image', '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', '/imu/filtered'),
            ('odom', '/odom'),
        ]
    )

    # RTAB-Map (subscribes to /odom and RGB-D to build 3D map with loop closures)
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rtabmap_ros'), 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'frame_id': 'base_link',
            'use_sim_time': 'True',
            'subscribe_depth': 'True',
            'subscribe_rgbd': 'False',
            'approx_sync': 'True',
            'rgb_topic': '/camera/image',
            'depth_topic': '/camera/depth_image',
            'camera_info_topic': '/camera/camera_info',
            'imu_topic': '/imu/filtered',
            'rtabmap_args': '--delete_db_on_start --Grid/FromDepth true --Grid/RangeMax 10.0 --Reg/Strategy 1 --Vis/CorType 2'
        }.items()
    )

    # RViz with pre-configured view
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('uav_slam_demo'), 'rviz', 'uav_slam_demo.rviz'])],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([gz, bridge, imu_filter, static_tf, rgbd_odom, rtabmap, rviz])
