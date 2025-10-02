# launch/random_forest_husky.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os, math, random, time

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

    # Robot spawn (read from launch args)
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))

    # Use system time for true randomization if seed = -1
    if seed == -1:
        seed = int(time.time() * 1000)
        print(f"[forest_random] Using random seed: {seed}")
    random.seed(seed)

    # Guardrail
    if max_sep <= min_sep:
        print(f"[forest_random] max_sep ({max_sep}) <= min_sep ({min_sep}); nudging max_sep slightly up.")
        max_sep = min_sep + 1e-6

    # Effective bounds
    xmin_eff, xmax_eff = xmin + margin, xmax - margin
    ymin_eff, ymax_eff = ymin + margin, ymax - margin

    # Tree sampler with exclusion zone
    def sample_points(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, seed_pts=1, max_tries=40000):
        pts, tries = [], 0
        min_r2, max_r2 = min_s**2, max_s**2
        spawn_exclusion_radius = 1.5  # meters
        spawn_r2 = spawn_exclusion_radius**2

        while len(pts) < n and tries < max_tries:
            x, y = random.uniform(xmin_, xmax_), random.uniform(ymin_, ymax_)

            # Exclude near Husky spawn
            if (x - spawn_x)**2 + (y - spawn_y)**2 <= spawn_r2:
                tries += 1
                continue

            # Enforce min separation
            if not all((x - xi)**2 + (y - yi)**2 >= min_r2 for xi, yi in pts):
                tries += 1
                continue

            # Allow cluster seeding
            if len(pts) < seed_pts:
                pts.append((x, y))
                continue

            # Enforce max separation
            if any((x - xi)**2 + (y - yi)**2 <= max_r2 for xi, yi in pts):
                pts.append((x, y))

            tries += 1
        return pts

    pts = sample_points(num_a + num_b, xmin_eff, xmax_eff, ymin_eff, ymax_eff, min_sep, max_sep, seed_pts=1)
    if len(pts) < num_a + num_b:
        print(f"[forest_random] placed {len(pts)}/{num_a+num_b} trees; relax min_sep/increase max_sep or enlarge bounds.")

    # Helper to generate <include>
    def inc(uri, name, x, y, z_, yaw):
        return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
    </include>"""

    # Build world SDF
    blocks = [inc("model://forest_env", "forest_env", 0, 0, 0, 0.0)]
    for i, (x, y) in enumerate(pts[:num_a]):
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0, 2*math.pi)))
    for j, (x, y) in enumerate(pts[num_a:]):
        blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z, random.uniform(0, 2*math.pi)))

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
    print(f"[husky_forest_random] wrote {out_path} with seed {seed} (spawn at {spawn_x:.2f}, {spawn_y:.2f})")
    return out_path

def launch_setup(context, *args, **kwargs):
    world_path = generate_forest_world(context)
    return [ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'], output='screen')]

def generate_launch_description():
    pkg_path = get_package_share_directory('john')
    resource_path = os.pathsep.join([
        os.path.join(pkg_path, 'models'),
        os.path.join(pkg_path, 'worlds')
    ])
    ld = LaunchDescription()

    # Resource paths
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Core args
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))

    # Forest args
    for arg in [
        ('num_a','18'), ('num_b','2'),
        ('xmin','-5.0'), ('xmax','5.0'),
        ('ymin','-5.0'), ('ymax','5.0'),
        ('margin','0.5'), ('min_sep','0.8'), ('max_sep','3'),
        ('z','0.0'), ('seed','-1')
    ]:
        ld.add_action(DeclareLaunchArgument(arg[0], default_value=arg[1]))

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Randomize Husky spawn and declare as args
    spawn_x = str(random.uniform(-2.0, 2.0))
    spawn_y = str(random.uniform(-2.0, 2.0))
    spawn_yaw = str(random.uniform(0, 2*math.pi))
    ld.add_action(DeclareLaunchArgument('spawn_x', default_value=spawn_x))
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value=spawn_y))
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value=spawn_yaw))

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str)

    # Nodes
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    ))
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    ))
    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic','/robot_description',
                   '-x', LaunchConfiguration('spawn_x'),
                   '-y', LaunchConfiguration('spawn_y'),
                   '-z','0.4',
                   '-Y', LaunchConfiguration('spawn_yaw'),
                   '-wait','5']
    ))
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([pkg_path,'config','gazebo_bridge.yaml']),
                     'use_sim_time': use_sim_time}],
        output='screen'
    ))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([pkg_path,'config','john.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,'launch','johnNavigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    ))

    # Generate + start world
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
