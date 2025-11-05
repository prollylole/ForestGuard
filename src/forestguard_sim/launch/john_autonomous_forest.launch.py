from launch import LaunchDescription, actions
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os, math, random, time

# ------------------------
# Forest world generation
# ------------------------
def generate_forest_world(context):
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

    # spawn (set in launch_setup)
    spawn_x = float(context.launch_configurations['resolved_spawn_x'])
    spawn_y = float(context.launch_configurations['resolved_spawn_y'])

    if seed == -1:
        seed = int(time.time() * 1000)
        print(f"[forest_random] Using random seed: {seed}")
    random.seed(seed)

    if max_sep <= min_sep:
        max_sep = min_sep + 1e-6

    xmin_eff, xmax_eff = xmin + margin, xmax - margin
    ymin_eff, ymax_eff = ymin + margin, ymax - margin

    def sample_points(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, max_tries=40000):
        pts, tries = [], 0
        min_r2, max_r2 = min_s * min_s, max_s * max_s
        spawn_r2 = 1.5 * 1.5
        while len(pts) < n and tries < max_tries:
            x = random.uniform(xmin_, xmax_)
            y = random.uniform(ymin_, ymax_)
            if (x - spawn_x)**2 + (y - spawn_y)**2 <= spawn_r2:
                tries += 1; continue
            if not all((x - xi)**2 + (y - yi)**2 >= min_r2 for xi, yi in pts):
                tries += 1; continue
            if len(pts) == 0 or any((x - xi)**2 + (y - yi)**2 <= max_r2 for xi, yi in pts):
                pts.append((x, y))
            tries += 1
        return pts

    pts = sample_points(num_a + num_b, xmin_eff, xmax_eff, ymin_eff, ymax_eff, min_sep, max_sep)

    def inc(uri, name, x, y, z_, yaw):
        return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
    </include>"""

    blocks = [inc("model://forest_env", "forest_env", 0, 0, 0, 0.0)]
    for i, (x, y) in enumerate(pts[:num_a]):
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0.0, 2*math.pi)))
    for j, (x, y) in enumerate(pts[num_a:num_a+num_b]):
        blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z, random.uniform(0.0, 2*math.pi)))

    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="forest_world">
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
    </light>
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>1 1 1 1</ambient><background>0.6 0.8 1.0 1</background></scene>
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
# Setup phase (returns gazebo + spawner + shutdown handler)
# ------------------------
def launch_setup(context, *args, **kwargs):
    # Resolve robot spawn
    spawn_x_val = resolve_spawn_arg(context, 'spawn_x', -2.0, 2.0)
    spawn_y_val = resolve_spawn_arg(context, 'spawn_y', -2.0, 2.0)
    spawn_yaw_val = resolve_spawn_arg(context, 'spawn_yaw', 0.0, 2*math.pi)

    # Expose to world generator
    context.launch_configurations['resolved_spawn_x'] = spawn_x_val
    context.launch_configurations['resolved_spawn_y'] = spawn_y_val

    # Generate world
    world_path = generate_forest_world(context)

    # Start Ignition GUI
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen',
    )

    # Shut down everything when Ignition GUI exits
    gazebo_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_process,
            on_exit=[Shutdown(reason='Ignition GUI closed')]
        )
    )

    # Spawn the robot
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

    # Return all actions created in setup
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

    # Gazebo resource paths
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    # Common args
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))
    ld.add_action(DeclareLaunchArgument('ui', default_value='True'))
    ld.add_action(DeclareLaunchArgument('teleop', default_value='True'))

    # Forest args
    for arg in [
        DeclareLaunchArgument('num_a', default_value='35'),
        DeclareLaunchArgument('num_b', default_value='2'),
        DeclareLaunchArgument('xmin', default_value='-7.0'),
        DeclareLaunchArgument('xmax', default_value='7.0'),
        DeclareLaunchArgument('ymin', default_value='-7.0'),
        DeclareLaunchArgument('ymax', default_value='7.0'),
        DeclareLaunchArgument('margin', default_value='0.5'),
        DeclareLaunchArgument('min_sep', default_value='2.25'),
        DeclareLaunchArgument('max_sep', default_value='3.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('spawn_x', default_value='RANDOM'),
        DeclareLaunchArgument('spawn_y', default_value='RANDOM'),
        DeclareLaunchArgument('spawn_yaw', default_value='RANDOM'),
    ]:
        ld.add_action(arg)

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
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        condition=IfCondition(LaunchConfiguration('teleop'))
    ))
    ld.add_action(Node(
        package='forestguard_ui', executable='twist_scaler', name='twist_scaler',
        parameters=[{'in_topic': '/cmd_vel_raw', 'out_topic': '/cmd_vel'}]
    ))

    # RViz (keep a handle so we can attach a shutdown handler)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'forest.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    ld.add_action(rviz_node)

    # Shut everything down when RViz exits
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

    # Generate world, start Gazebo, and add its shutdown handler + spawner
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
