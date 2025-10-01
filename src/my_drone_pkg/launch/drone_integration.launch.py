# File: forest_random_with_drone_gz.launch.py
#
# 1) Generate forest world SDF (paused) at ~/.ros/forest_random.world.sdf
# 2) Start Gazebo Sim via the "gz" launcher (gz sim <world>)
# 3) Bring up URDF (robot_state_publisher), optional bridge + RViz
# 4) Spawn drone into /world/forest_world
#
# Requires:
#  - ros_gz_sim installed (we Include its gz launcher, or your custom one)
#  - my_drone_pkg (URDF + bridge YAML)
#  - forest_guard_sim (models for trees/terrain)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction,
    IncludeLaunchDescription, TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os, math, random
import xacro

# -------------------------- WORLD GENERATOR --------------------------
def _gen_and_write_world(context, *args, **kwargs):
    num_a   = int(LaunchConfiguration('num_a').perform(context))
    num_b   = int(LaunchConfiguration('num_b').perform(context))
    xmin    = float(LaunchConfiguration('xmin').perform(context))
    xmax    = float(LaunchConfiguration('xmax').perform(context))
    ymin    = float(LaunchConfiguration('ymin').perform(context))
    ymax    = float(LaunchConfiguration('ymax').perform(context))
    min_sep = float(LaunchConfiguration('min_sep').perform(context))
    max_sep = float(LaunchConfiguration('max_sep').perform(context))
    margin  = float(LaunchConfiguration('margin').perform(context))
    z       = float(LaunchConfiguration('z').perform(context))
    seed    = int(LaunchConfiguration('seed').perform(context))
    random.seed(seed)

    if max_sep <= min_sep:
        print(f"[forest_random] max_sep ({max_sep}) <= min_sep ({min_sep}); nudging max_sep slightly up.")
        max_sep = min_sep + 1e-6

    xmin_eff = xmin + margin
    xmax_eff = xmax - margin
    ymin_eff = ymin + margin
    ymax_eff = ymax - margin

    def sample_points(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, seed_pts=1, max_tries=40000):
        pts, tries = [], 0
        min_r2 = min_s * min_s
        max_r2 = max_s * max_s
        while len(pts) < n and tries < max_tries:
            x = random.uniform(xmin_, xmax_)
            y = random.uniform(ymin_, ymax_)
            if not all((x - xi)**2 + (y - yi)**2 >= min_r2 for xi, yi in pts):
                tries += 1; continue
            if len(pts) < seed_pts:
                pts.append((x, y)); continue
            if any((x - xi)**2 + (y - yi)**2 <= max_r2 for xi, yi in pts):
                pts.append((x, y))
            tries += 1
        return pts

    total = num_a + num_b
    pts = sample_points(total, xmin_eff, xmax_eff, ymin_eff, ymax_eff, min_sep, max_sep)
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
    # Ground model and trees (URIs must be model://<model_name>/...)
    blocks.append(inc("model://forest_env", "forest_env", 0, 0, 0, 0.0))
    for i, (x, y) in enumerate(pts[:num_a]):
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0.0, 2*math.pi)))
    for j, (x, y) in enumerate(pts[num_a:num_a+num_b]):
        blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z, random.uniform(0.0, 2*math.pi)))

    # Start paused via <gui><start_paused>true</start_paused></gui>
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
    <gui>
      <start_paused>true</start_paused>
    </gui>
{"".join(blocks)}
  </world>
</sdf>
"""
    out_dir = os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'forest_random.world.sdf')
    with open(out_path, 'w') as f:
        f.write(world_sdf)
    print(f"[forest_random] wrote {out_path}")

    # Nothing to return; the IncludeLaunchDescription below will consume this path via LaunchConfiguration
    return []

# -------------------------- MAIN LAUNCH --------------------------
def generate_launch_description():
    # Packages’ share
    pkg_env_share   = get_package_share_directory('forest_guard_sim')
    pkg_drone_share = get_package_share_directory('my_drone_pkg')
    pkg_gz_share    = get_package_share_directory('ros_gz_sim')  # we’ll include its gz launcher

    # Absolute resource roots (dedup and only keep existing)
    env_models   = os.path.join(pkg_env_share,   'models')
    env_worlds   = os.path.join(pkg_env_share,   'worlds')
    drone_models = os.path.join(pkg_drone_share, 'models')
    drone_worlds = os.path.join(pkg_drone_share, 'worlds')  # ok if missing
    home_models  = os.path.join(os.environ.get('HOME', ''), 'ForestGuard', 'src', 'my_drone_pkg', 'models')

    def existing_unique(paths):
        seen, out = set(), []
        for p in paths:
            if p and os.path.isdir(p) and p not in seen:
                seen.add(p); out.append(p)
        return out

    resource_paths = existing_unique([drone_models, drone_worlds, env_models, env_worlds, home_models])
    gz_full  = ':'.join([p for p in ([os.environ.get('GZ_SIM_RESOURCE_PATH', '')] + resource_paths) if p])
    ign_full = ':'.join([p for p in ([os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')] + resource_paths) if p])

    # -------------------------- Arguments --------------------------
    args = [
        # forest tunables
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

        # drone pose + toggles
        DeclareLaunchArgument('robot_name', default_value='test_drone'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z_spawn', default_value='0.5'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('bring_bridge', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # gz launcher args (we’ll fill gz_args dynamically with the generated world path)
        DeclareLaunchArgument('gz_version', default_value='6'),
        DeclareLaunchArgument('gz_args',    default_value=''),  # will be set via a Timer after world is written
    ]

    # -------------------------- Set env once (clean) --------------------------
    set_gz_res   = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_full)
    set_ign_res  = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_full)
    set_gz_plug  = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ':'.join(filter(None, [os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                               '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins']))
    )
    set_ign_plug = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        ':'.join(filter(None, [os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
                               '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins']))
    )

    # Debug print (once)
    dbg = ExecuteProcess(
        cmd=['bash', '-lc',
             'echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"; '
             'echo "IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH"; '
             f'ls -ld "{env_models}"   || true; '
             f'ls -ld "{env_worlds}"   || true; '
             f'ls -ld "{drone_models}" || true; '
             f'ls -ld "{drone_worlds}" || true; '
             f'ls -ld "{home_models}"  || true; '],
        output='screen'
    )

    # -------------------------- Generate world first --------------------------
    gen_world = OpaqueFunction(function=_gen_and_write_world)

    # Path to the generated world (as we know where we wrote it)
    world_path = os.path.join(os.path.expanduser('~'), '.ros', 'forest_random.world.sdf')

    # -------------------------- Include the gz launcher --------------------------
    # We call the official ros_gz_sim gz_sim.launch.py (has the exact interface you pasted).
    gz_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Start **paused**: just pass the world path (no "-r").
        launch_arguments={
            'gz_args': world_path,
            'gz_version': LaunchConfiguration('gz_version'),
            # optional toggles available in that launcher:
            'debugger': 'false',
            'debug_env': 'false',
            'on_exit_shutdown': 'false',
        }.items()
    )

    # -------------------------- Robot bring-up --------------------------
    # URDF (xacro -> XML)
    xacro_path = os.path.join(pkg_drone_share, 'urdf', 'X4.urdf.xacro')
    try:
        robot_description_xml = xacro.process_file(xacro_path).toxml()
    except Exception as e:
        robot_description_xml = "<robot name='x4_placeholder'/>"
        print(f"[launch] xacro processing failed: {e}")

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, {'robot_description': robot_description_xml}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_drone_share, 'config', 'ros_gz_bridge_config.yaml')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('bring_bridge'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_drone_share, 'config', 'drone.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn the drone into the world after gz is up
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-world', 'forest_world',
            '-name',  LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z_spawn'),
            '-Y', LaunchConfiguration('Y'),
            # Use URDF string; if your SDF is fully correct, you can swap to `-file` model.sdf
            '-string', robot_description_xml,
            '-allow_renaming', 'true'
        ]
    )
    delayed_spawn = TimerAction(period=2.0, actions=[spawn])

    # -------------------------- Assemble --------------------------
    return LaunchDescription(
        args + [
            set_gz_res, set_ign_res, set_gz_plug, set_ign_plug, dbg,
            gen_world,            # write the world file first
            gz_launcher,          # then start gz sim with that world (paused)
            joint_state_publisher,
            robot_state_publisher,
            bridge,
            rviz,
            delayed_spawn
        ]
    )
