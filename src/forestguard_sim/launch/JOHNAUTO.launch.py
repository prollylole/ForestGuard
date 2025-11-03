from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os, math, random

# ------------------------
# Tunables (defaults)
# ------------------------
N_GOOD = 35
N_BAD  = 5
MIN_SEP = 3.0           # metres (minimum centre-to-centre)
MAX_NEIGHBOR = 6.0      # soft cap: each new point should have someone within this distance
SPAWN_EXCL_RADIUS = 1.5 # keep trees away from spawn (m)
AREA_HALF = 9.0         # layout box: [-AREA_HALF, AREA_HALF] in x & y
TREE_Z = 0.0            # tree z placement
DEFAULT_SEED = 424242   # fixed seed -> same layout every run

# ------------------------
# Deterministic Poisson-disk style sampler
# ------------------------
def poisson_disk_deterministic(n_target, xmin, xmax, ymin, ymax,
                               min_sep, max_neighbor, spawn_xy, excl_r,
                               rng, k=30):
    # Bridson-ish sampler in 2D with a soft "must-have-a-neighbor" rule
    w = max(xmax - xmin, 1e-6); h = max(ymax - ymin, 1e-6)
    r = min_sep
    cell = r / math.sqrt(2)
    cols = int(math.ceil(w / cell))
    rows = int(math.ceil(h / cell))
    grid = [[-1]*cols for _ in range(rows)]
    samples, active = [], []

    def in_bounds(x, y):
        return xmin <= x <= xmax and ymin <= y <= ymax

    def too_close(x, y):
        gx = int((x - xmin) / cell); gy = int((y - ymin) / cell)
        for j in range(max(0, gy-2), min(rows, gy+3)):
            for i in range(max(0, gx-2), min(cols, gx+3)):
                idx = grid[j][i]
                if idx != -1:
                    sx, sy = samples[idx]
                    if (x - sx)**2 + (y - sy)**2 < r*r:
                        return True
        return False

    def isolated_from_all_neighbors(x, y):
        if not samples:
            return False
        max2 = max_neighbor*max_neighbor
        for sx, sy in samples:
            if (x - sx)**2 + (y - sy)**2 <= max2:
                return False
        return True

    sx0, sy0 = spawn_xy
    # initial point
    for _ in range(1000):
        x0 = rng.uniform(xmin, xmax)
        y0 = rng.uniform(ymin, ymax)
        if (x0 - sx0)**2 + (y0 - sy0)**2 <= excl_r*excl_r:
            continue
        if not too_close(x0, y0):
            samples.append((x0, y0))
            gx0 = int((x0 - xmin) / cell); gy0 = int((y0 - ymin) / cell)
            grid[gy0][gx0] = 0
            active.append(0)
            break

    while active and len(samples) < n_target:
        idx = rng.choice(active)
        sx, sy = samples[idx]
        found = False
        for _ in range(k):
            rho = rng.uniform(r, 2*r)
            ang = rng.uniform(0, 2*math.pi)
            x = sx + rho*math.cos(ang)
            y = sy + rho*math.sin(ang)
            if not in_bounds(x, y):
                continue
            if (x - sx0)**2 + (y - sy0)**2 <= excl_r*excl_r:
                continue
            if too_close(x, y):
                continue
            if isolated_from_all_neighbors(x, y):
                continue
            samples.append((x, y))
            gx = int((x - xmin) / cell); gy = int((y - ymin) / cell)
            grid[gy][gx] = len(samples)-1
            active.append(len(samples)-1)
            found = True
            break
        if not found:
            active.remove(idx)

    # Top-up if needed (rare) with uniform tries
    tries = 0
    while len(samples) < n_target and tries < 20000:
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        if (x - sx0)**2 + (y - sy0)**2 <= excl_r*excl_r:
            tries += 1; continue
        if too_close(x, y) or isolated_from_all_neighbors(x, y):
            tries += 1; continue
        samples.append((x, y))
        tries += 1

    # round for nicer SDF
    return [(round(px, 3), round(py, 3)) for (px, py) in samples[:n_target]]

# ------------------------
# World SDF builder
# ------------------------
def make_world_sdf(tree_positions, n_good=N_GOOD, z_tree=TREE_Z):
    def inc(uri, name, x, y, z_, yaw):
        return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
    </include>"""

    blocks = []
    # Terrain model (your forest_env)
    blocks.append(inc("model://forest_env", "forest_env", 0, 0, 0, 0.0))

    # Trees: first n_good are tree1 (good), remainder are tree2 (bad)
    for i, (x, y) in enumerate(tree_positions):
        if i < n_good:
            blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z_tree, 0.0))
        else:
            j = i - n_good
            blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z_tree, 0.0))

    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="forest_world">
    <!-- Physics & timing -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <!-- Scene lighting -->
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.6 0.8 1.0 1</background>
    </scene>

    <!-- Directional sun -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Terrain and trees -->
{''.join(blocks)}
  </world>
</sdf>"""

# ------------------------
# World writer (deterministic-random layout)
# ------------------------
def write_world(context):
    # read spawn for exclusion zone anchor
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
    # seed: default fixed, but overridable
    seed = int(LaunchConfiguration('seed').perform(context))
    rng = random.Random(seed)

    xmin = -AREA_HALF; xmax = AREA_HALF
    ymin = -AREA_HALF; ymax = AREA_HALF

    positions = poisson_disk_deterministic(
        n_target=N_GOOD + N_BAD,
        xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax,
        min_sep=MIN_SEP, max_neighbor=MAX_NEIGHBOR,
        spawn_xy=(spawn_x, spawn_y), excl_r=SPAWN_EXCL_RADIUS,
        rng=rng, k=30
    )

    world_sdf = make_world_sdf(positions, n_good=N_GOOD, z_tree=TREE_Z)

    out_dir = os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'husky_forest_drand.world.sdf')
    with open(out_path, 'w') as f:
        f.write(world_sdf)
    print(f"[husky_forest_drand] wrote {out_path} with seed {seed}")
    return out_path

# ------------------------
# Setup: write world, start Ignition, spawn robot
# ------------------------
def launch_setup(context, *args, **kwargs):
    # explicit spawn pose (no RANDOM)
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context))

    world_path = write_world(context)

    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen',
    )

    gazebo_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_process,
            on_exit=[Shutdown(reason='Ignition GUI closed')]
        )
    )

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-topic', '/robot_description',
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', '0.4',
            '-Y', str(spawn_yaw),
            '-wait', '5'
        ]
    )

    return [gazebo_process, gazebo_shutdown_handler, robot_spawner]

# ------------------------
# Main launch description
# ------------------------
def generate_launch_description():
    pkg_path = get_package_share_directory('john')
    resource_path = os.pathsep.join([
        os.path.join(pkg_path, 'models'),
        os.path.join(pkg_path, 'worlds')
    ])

    ld = LaunchDescription()

    # Gazebo resource paths (to find forest_env, tree1, tree2)
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Basic toggles
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))
    ld.add_action(DeclareLaunchArgument('ui', default_value='True'))
    ld.add_action(DeclareLaunchArgument('teleop', default_value='True'))

    # Spawn pose (deterministic; defaults to 0)
    ld.add_action(DeclareLaunchArgument('spawn_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value='0.0'))

    # Deterministic "randomness": fixed default seed, can override if desired
    ld.add_action(DeclareLaunchArgument('seed', default_value=str(DEFAULT_SEED)))

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # EKF
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    ))

    # Bridge
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge.yaml']),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    ))

    # UI + Teleop
    ld.add_action(Node(
        package='forestguard_ui', executable='run_ui', output='screen',
        condition=IfCondition(LaunchConfiguration('ui'))
    ))
    ld.add_action(Node(
        package='forestguard_ui', executable='controller_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'xbox_teleop.yaml'])],
        remappings=[('/cmd_vel', '/cmd_vel')],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'forest.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    ld.add_action(rviz_node)

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[Shutdown(reason='RViz closed')]
        )
    ))

    # Nav2 (optional)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_path, 'launch', 'johnNavigation.launch.py'])]
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    ))

    # Write world, start Gazebo, spawn robot
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
