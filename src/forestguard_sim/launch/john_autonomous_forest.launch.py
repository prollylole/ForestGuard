from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os, math, random, time


# ------------------------
# Forest world generator
# ------------------------
def generate_forest_world(context):
    num_a = int(LaunchConfiguration('num_a').perform(context))
    num_b = int(LaunchConfiguration('num_b').perform(context))
    xmin = float(LaunchConfiguration('xmin').perform(context))
    xmax = float(LaunchConfiguration('xmax').perform(context))
    ymin = float(LaunchConfiguration('ymin').perform(context))
    ymax = float(LaunchConfiguration('ymax').perform(context))
    min_sep = float(LaunchConfiguration('min_sep').perform(context))
    margin = float(LaunchConfiguration('margin').perform(context))
    z = float(LaunchConfiguration('z').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))

    # resolved spawn from context (set in launch_setup)
    spawn_x = float(context.launch_configurations['resolved_spawn_x'])
    spawn_y = float(context.launch_configurations['resolved_spawn_y'])

    if seed == -1:
        seed = int(time.time() * 1000)
        print(f"[forest_random] Using random seed: {seed}")
    random.seed(seed)

    xmin_eff, xmax_eff = xmin + margin, xmax - margin
    ymin_eff, ymax_eff = ymin + margin, ymax - margin

    pts = []
    while len(pts) < num_a + num_b:
        x = random.uniform(xmin_eff, xmax_eff)
        y = random.uniform(ymin_eff, ymax_eff)
        if all((x - xi)**2 + (y - yi)**2 >= min_sep**2 for xi, yi in pts):
            pts.append((x, y))

    def inc(uri, name, x, y, z_, yaw):
        return f"""
        <include>
          <uri>{uri}</uri>
          <name>{name}</name>
          <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
        </include>"""

    blocks = ["<include><uri>model://forest_env</uri><name>forest_env</name><pose>0 0 0 0 0 0</pose></include>"]
    # tree1s
    for i, (x, y) in enumerate(pts[:num_a]):
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0.0, 2*math.pi)))
    # tree2s
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
    world_path = os.path.join(out_dir, 'husky_autonomous.world.sdf')
    with open(world_path, 'w') as f:
        f.write(world_sdf)
    print(f"[forest_autonomous] wrote {world_path} with seed {seed}")
    return world_path


def resolve_spawn_arg(context, name, lo, hi):
    val = LaunchConfiguration(name).perform(context)
    return str(random.uniform(lo, hi)) if val == 'RANDOM' else val


def launch_setup(context, *args, **kwargs):
    spawn_x = resolve_spawn_arg(context, 'spawn_x', -2.0, 2.0)
    spawn_y = resolve_spawn_arg(context, 'spawn_y', -2.0, 2.0)
    spawn_yaw = resolve_spawn_arg(context, 'spawn_yaw', 0.0, 2*math.pi)
    context.launch_configurations['resolved_spawn_x'] = spawn_x
    context.launch_configurations['resolved_spawn_y'] = spawn_y

    world_path = generate_forest_world(context)

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.4', '-Y', spawn_yaw
        ]
    )
    return [gazebo, spawner]


def generate_launch_description():
    pkg_path = get_package_share_directory('john')

    # Add Ignition/Gazebo resource paths so model:// URIs resolve (forest_env, tree1, tree2)
    models_dir  = os.path.join(pkg_path, 'models')
    worlds_dir  = os.path.join(pkg_path, 'worlds')
    resource_path = os.pathsep.join([models_dir, worlds_dir])

    ld = LaunchDescription()

    # Export env for both classic and new Ignition variables
    existing_ign = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    existing_gz  = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_gazebo_model = os.environ.get('GAZEBO_MODEL_PATH', '')

    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=(resource_path if not existing_ign else resource_path + os.pathsep + existing_ign)
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(resource_path if not existing_gz else resource_path + os.pathsep + existing_gz)
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=(resource_path if not existing_gazebo_model else resource_path + os.pathsep + existing_gazebo_model)
    ))

    # ------------------ Arguments ------------------
    for arg in [
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('ui', default_value='True'),
        DeclareLaunchArgument('teleop', default_value='True'),
        DeclareLaunchArgument('use_map', default_value='True'),
        DeclareLaunchArgument('num_a', default_value='25'),
        DeclareLaunchArgument('num_b', default_value='2'),
        DeclareLaunchArgument('xmin', default_value='-7.0'),
        DeclareLaunchArgument('xmax', default_value='7.0'),
        DeclareLaunchArgument('ymin', default_value='-7.0'),
        DeclareLaunchArgument('ymax', default_value='7.0'),
        DeclareLaunchArgument('margin', default_value='0.5'),
        DeclareLaunchArgument('min_sep', default_value='1.25'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('spawn_x', default_value='RANDOM'),
        DeclareLaunchArgument('spawn_y', default_value='RANDOM'),
        DeclareLaunchArgument('spawn_yaw', default_value='RANDOM'),
    ]:
        ld.add_action(arg)

    # ------------------ Core Nodes ------------------
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

    # EKF localization (expects /odometry & /imu from bridge; publishes /odometry/filtered & TF)
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'robot_localization.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    ))

    # Bridge Gazebo ↔ ROS (topics configured in your gazebo_bridge.yaml)
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge.yaml'])}],
        output='screen'
    ))

    # Static transform map→odom (AMCL will localize, but we keep map->odom static zero unless you want AMCL to publish)
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    ))

    # ------------------ Nav2 stack ------------------
    # include your navigation_map.launch.py (which now defaults to john/config paths)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_path, 'launch', 'navigation_map.launch.py'])]
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('use_map'))
    ))

    # ------------------ UI + Teleop ------------------
    ld.add_action(Node(
        package='turtlebot_ui', executable='run_ui', output='screen',
        condition=IfCondition(LaunchConfiguration('ui'))
    ))
    ld.add_action(Node(
        package='turtlebot_ui', executable='controller_bridge', output='screen',
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
        package='turtlebot_ui', executable='twist_scaler', name='twist_scaler',
        parameters=[{'in_topic': '/cmd_vel_raw', 'out_topic': '/cmd_vel'}]
    ))

    # ------------------ RViz ------------------
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'forest.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))

    # ------------------ Gazebo + Spawn ------------------
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
