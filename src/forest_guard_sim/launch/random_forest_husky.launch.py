# launch/random_forest_husky.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os, math, random

def generate_forest_world(context):
    # Read CLI args for forest generation
    num_a = int(LaunchConfiguration('num_a').perform(context))
    num_b = int(LaunchConfiguration('num_b').perform(context))
    xmin = float(LaunchConfiguration('xmin').perform(context))
    xmax = float(LaunchConfiguration('xmax').perform(context))
    ymin = float(LaunchConfiguration('ymin').perform(context))
    ymax = float(LaunchConfiguration('ymax').perform(context))
    min_sep = float(LaunchConfiguration('min_sep').perform(context))
    max_sep = float(LaunchConfiguration('max_sep').perform(context))
    margin = float(LaunchConfiguration('margin').perform(context))
    z = float(LaunchConfiguration('z').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    random.seed(seed)

    # Guardrail: ensure feasible constraints
    if max_sep <= min_sep:
        print(f"[forest_random] max_sep ({max_sep}) <= min_sep ({min_sep}); nudging max_sep slightly up.")
        max_sep = min_sep + 1e-6

    # Effective bounds (keep-off margin from edges)
    xmin_eff = xmin + margin
    xmax_eff = xmax - margin
    ymin_eff = ymin + margin
    ymax_eff = ymax - margin

    # Sampler with BOTH min and max separation (clusters)
    def sample_points(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, seed_pts=1, max_tries=40000):
        pts, tries = [], 0
        min_r2 = min_s * min_s
        max_r2 = max_s * max_s
        while len(pts) < n and tries < max_tries:
            x = random.uniform(xmin_, xmax_)
            y = random.uniform(ymin_, ymax_)
            # Enforce min distance from ALL placed points
            if not all((x - xi)**2 + (y - yi)**2 >= min_r2 for xi, yi in pts):
                tries += 1
                continue
            # Allow a seed to start the first cluster
            if len(pts) < seed_pts:
                pts.append((x, y))
                continue
            # Every subsequent point must be within max_sep of at least one existing point
            if any((x - xi)**2 + (y - yi)**2 <= max_r2 for xi, yi in pts):
                pts.append((x, y))
            tries += 1
        return pts

    total = num_a + num_b
    pts = sample_points(total, xmin_eff, xmax_eff, ymin_eff, ymax_eff, min_sep, max_sep, seed_pts=1)
    if len(pts) < total:
        print(f"[forest_random] placed {len(pts)}/{total} trees; relax min_sep/increase max_sep or enlarge bounds.")

    def inc(uri, name, x, y, z_, yaw):
        return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
    </include>"""

    blocks = []
    # Ground environment
    blocks.append(inc("model://forest_env", "forest_env", 0, 0, 0, 0.0))
    # First num_a -> tree1
    for i, (x, y) in enumerate(pts[:num_a]):
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0.0, 2*math.pi)))
    # Remaining -> tree2
    for j, (x, y) in enumerate(pts[num_a:num_a+num_b]):
        blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z, random.uniform(0.0, 2*math.pi)))

    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="forest_world">
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>
{"".join(blocks)}
  </world>
</sdf>"""
    
    out_dir = os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'husky_forest_random.world.sdf')
    with open(out_path, 'w') as f:
        f.write(world_sdf)
    print(f"[husky_forest_random] wrote {out_path}")
    
    return out_path

def launch_setup(context, *args, **kwargs):
    world_path = generate_forest_world(context)
    
    # Start Gazebo with the generated world
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen'
    )
    
    return [gazebo_process]

def generate_launch_description():
    # Get package path - ONLY forest_husky package
    pkg_path = get_package_share_directory('john')
    
    # Set resource paths - only forest_husky models
    resource_path = os.pathsep.join([
        os.path.join(pkg_path, 'models'),
        os.path.join(pkg_path, 'worlds')
    ])

    ld = LaunchDescription()
    
    # Environment variables for model resolution
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Launch arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Forest generation arguments
    forest_args = [
        DeclareLaunchArgument('num_a',   default_value='18'),
        DeclareLaunchArgument('num_b',   default_value='2'),
        DeclareLaunchArgument('xmin',    default_value='-5.0'),
        DeclareLaunchArgument('xmax',    default_value='5.0'),
        DeclareLaunchArgument('ymin',    default_value='-5.0'),
        DeclareLaunchArgument('ymax',    default_value='5.0'),
        DeclareLaunchArgument('margin',  default_value='0.5'),
        DeclareLaunchArgument('min_sep', default_value='0.8'),
        DeclareLaunchArgument('max_sep', default_value='3'),
        DeclareLaunchArgument('z',       default_value='0.0'),
        DeclareLaunchArgument('seed',    default_value='42'),
    ]
    
    for arg in forest_args:
        ld.add_action(arg)

    # Load robot_description from URDF xacro file
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str)
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }])
    ld.add_action(robot_state_publisher_node)

    # Robot localization
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Spawn robot in Gazebo - using ros_gz_sim (new package name)
    robot_spawner = Node(
        package='ros_gz_sim',  # Updated from ros_ign_gazebo
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4', '-wait', '5']  # Added wait for Gazebo to be ready
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2 - using ros_gz_bridge (new package name)
    gazebo_bridge = Node(
        package='ros_gz_bridge',  # Updated from ros_ign_bridge
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    ld.add_action(gazebo_bridge)

    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'john.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 navigation
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', 'johnNavigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # Add the opaque function that generates and launches the world
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld