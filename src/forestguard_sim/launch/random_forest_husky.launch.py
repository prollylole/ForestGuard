from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import actions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os, math, random, time

# ------------------------
# Forest world generation
# ------------------------
def generate_forest_world(context):
    # Read CLI args
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

    # Spawn position (already resolved in launch_setup)
    spawn_x = float(context.launch_configurations['resolved_spawn_x'])
    spawn_y = float(context.launch_configurations['resolved_spawn_y'])

    # Seed handling
    if seed == -1:
        seed = int(time.time() * 1000)
        print(f"[forest_random] Using random seed: {seed}")
    random.seed(seed)

    # Guardrail
    if max_sep <= min_sep:
        print(f"[forest_random] max_sep ({max_sep}) <= min_sep ({min_sep}); nudging max_sep slightly up.")
        max_sep = min_sep + 1e-6

    # Effective bounds
    xmin_eff = xmin + margin
    xmax_eff = xmax - margin
    ymin_eff = ymin + margin
    ymax_eff = ymax - margin

    # Sampler with min/max separation and a spawn exclusion around the robot
    def sample_points(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, seed_pts=1, max_tries=40000):
        pts, tries = [], 0
        min_r2 = min_s * min_s
        max_r2 = max_s * max_s
        spawn_exclusion_radius = 1.5
        spawn_r2 = spawn_exclusion_radius * spawn_exclusion_radius

        while len(pts) < n and tries < max_tries:
            x = random.uniform(xmin_, xmax_)
            y = random.uniform(ymin_, ymax_)

            # Keep away from robot spawn
            if (x - spawn_x)**2 + (y - spawn_y)**2 <= spawn_r2:
                tries += 1
                continue

            # Enforce min distance between trees
            if not all((x - xi)**2 + (y - yi)**2 >= min_r2 for xi, yi in pts):
                tries += 1
                continue

            # Seed first cluster
            if len(pts) < seed_pts:
                pts.append((x, y))
                continue

            # Subsequent points must be within max_sep of something to create clumps
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
    blocks.append(inc("model://forest_env", "forest_env", 0, 0, 0, 0.0))
    for i, (x, y) in enumerate(pts[:num_a]):
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0.0, 2*math.pi)))
    for j, (x, y) in enumerate(pts[num_a:num_a+num_b]):
        blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z, random.uniform(0.0, 2*math.pi)))

    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="forest_world">
    <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />
    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />
    <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />
    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>

      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <physics name="default_physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <scene>
      <ambient>1.0 1.0 1.0 1</ambient>
      <background>0.6 0.8 1.0 1</background>
      <shadows>0</shadows>
      <grid>0</grid>
    </scene>

    <spherical_coordinates>
      <latitude_deg>0.0</latitude_deg>
      <longitude_deg>0.0</longitude_deg>
      <elevation>10.0</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
{"".join(blocks)}
  </world>
</sdf>"""

    out_dir = os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'husky_forest_random.world.sdf')
    with open(out_path, 'w') as f:
        f.write(world_sdf)
    print(f"[husky_forest_random] wrote {out_path} with seed {seed}")
    return out_path

# ------------------------
# Helpers
# ------------------------
def resolve_spawn_arg(context, name, lo, hi):
    val = LaunchConfiguration(name).perform(context)
    return str(random.uniform(lo, hi)) if val == 'RANDOM' else val

# ------------------------
# Setup phase: resolve spawn, write world, start sim & spawn robot
# ------------------------
def launch_setup(context, *args, **kwargs):
    # Resolve robot spawn (RANDOM or numeric strings)
    spawn_x_val = resolve_spawn_arg(context, 'spawn_x', -2.0, 2.0)
    spawn_y_val = resolve_spawn_arg(context, 'spawn_y', -2.0, 2.0)
    spawn_yaw_val = resolve_spawn_arg(context, 'spawn_yaw', 0.0, 2*math.pi)

    # Make spawn available for world generation (tree exclusion)
    context.launch_configurations['resolved_spawn_x'] = spawn_x_val
    context.launch_configurations['resolved_spawn_y'] = spawn_y_val

    # Generate world file with trees
    world_path = generate_forest_world(context)

    # Start Gazebo with that world
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen',
        on_exit=actions.Shutdown(),
    )

    # Spawn robot here (we have concrete strings now)
    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-topic', '/robot_description',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', '0.4',
            '-Y', spawn_yaw_val,
            '-wait', '5'
        ]
    )

    return [gazebo_process, robot_spawner]

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

    # Gazebo resource paths
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Common args
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))

    # STEP 5: UI toggle
    ld.add_action(DeclareLaunchArgument('ui', default_value='True'))

    # Forest args
    for arg in [
        DeclareLaunchArgument('num_a', default_value='25'),
        DeclareLaunchArgument('num_b', default_value='2'),
        DeclareLaunchArgument('xmin', default_value='-7.0'),
        DeclareLaunchArgument('xmax', default_value='7.0'),
        DeclareLaunchArgument('ymin', default_value='-7.0'),
        DeclareLaunchArgument('ymax', default_value='7.0'),
        DeclareLaunchArgument('margin', default_value='0.5'),
        DeclareLaunchArgument('min_sep', default_value='1.25'),
        DeclareLaunchArgument('max_sep', default_value='4.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='-1'),  # -1 => time-based seed
    ]:
        ld.add_action(arg)

    # Husky spawn args (RANDOM or numeric strings)
    ld.add_action(DeclareLaunchArgument('spawn_x', default_value='RANDOM'))
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value='RANDOM'))
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value='RANDOM'))

    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # Localization
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    ld.add_action(robot_localization_node)

    # Bridge
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge.yaml']),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    ld.add_action(gazebo_bridge)

    ui_node = Node(
        package='forestguard_ui',
        executable='run_ui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('ui'))
    )

    controller_bridge = Node(
        package='forestguard_ui',
        executable='controller_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    )

    ld.add_action(ui_node)
    ld.add_action(controller_bridge)

    # ------------------------------
    # Joystick Teleoperation
    # ------------------------------
    teleop_joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }],
        condition=IfCondition(LaunchConfiguration('teleop'))
    )

    teleop_twist_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'xbox_teleop.yaml'])],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        condition=IfCondition(LaunchConfiguration('teleop'))
    )

    twist_scaler = Node(
        package='forestguard_ui',
        executable='twist_scaler',
        name='twist_scaler',
        parameters=[{'in_topic': '/cmd_vel_raw', 'out_topic': '/cmd_vel'}],
)
    ld.add_action(teleop_joy_node)
    ld.add_action(teleop_twist_node)
    ld.add_action(twist_scaler)

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'john.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        on_exit=actions.Shutdown(),
    )
    ld.add_action(rviz_node)

    # Nav2 (optional)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_path, 'launch', 'johnNavigation.launch.py'])]
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # Generate world + start Gazebo + spawn robot
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
