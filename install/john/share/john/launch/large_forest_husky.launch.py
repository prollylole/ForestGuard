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
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os, math, random, time

# ------------------------
# Large Forest world generation
# ------------------------
def generate_large_forest_world(context):
    # Read CLI args - 4x larger area and 4x more trees
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

    # Seed handling - use fixed seed for static forest
    if seed == -1:
        seed = 42  # Fixed seed for reproducible forest
    random.seed(seed)

    print(f"[large_forest] Generating forest with seed: {seed}")
    print(f"[large_forest] Area: {xmin:.1f} to {xmax:.1f} (x), {ymin:.1f} to {ymax:.1f} (y)")
    print(f"[large_forest] Trees: {num_a} type A + {num_b} type B = {num_a + num_b} total")

    # Guardrail
    if max_sep <= min_sep:
        print(f"[large_forest] max_sep ({max_sep}) <= min_sep ({min_sep}); nudging max_sep slightly up.")
        max_sep = min_sep + 1e-6

    # Effective bounds (with margin)
    xmin_eff = xmin + margin
    xmax_eff = xmax - margin
    ymin_eff = ymin + margin
    ymax_eff = ymax - margin

    # Improved tree placement algorithm
    def sample_points_improved(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, spawn_x_, spawn_y_, max_tries=100000):
        pts = []
        min_r2 = min_s * min_s
        max_r2 = max_s * max_s
        spawn_exclusion_radius = 3.0  # Larger exclusion zone
        spawn_r2 = spawn_exclusion_radius * spawn_exclusion_radius
        
        # Create grid for more uniform distribution
        grid_size = max_s
        grid = {}
        
        def get_grid_cell(x, y):
            return (int(x / grid_size), int(y / grid_size))
        
        def is_valid_position(x, y, pts_list):
            # Check spawn exclusion
            if (x - spawn_x_)**2 + (y - spawn_y_)**2 <= spawn_r2:
                return False
            
            # Check grid neighbors first for performance
            cell = get_grid_cell(x, y)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    neighbor_cell = (cell[0] + dx, cell[1] + dy)
                    if neighbor_cell in grid:
                        for pt in grid[neighbor_cell]:
                            dist2 = (x - pt[0])**2 + (y - pt[1])**2
                            if dist2 < min_r2:
                                return False
            
            # Detailed check for all points
            for pt in pts_list:
                dist2 = (x - pt[0])**2 + (y - pt[1])**2
                if dist2 < min_r2:
                    return False
            return True
        
        tries = 0
        while len(pts) < n and tries < max_tries:
            # Use different sampling strategies
            if len(pts) < n * 0.7:  # First 70%: uniform random
                x = random.uniform(xmin_, xmax_)
                y = random.uniform(ymin_, ymax_)
            else:  # Last 30%: grid-based for better coverage
                grid_cells_x = int((xmax_ - xmin_) / (min_s * 1.5))
                grid_cells_y = int((ymax_ - ymin_) / (min_s * 1.5))
                cell_x = tries % grid_cells_x
                cell_y = (tries // grid_cells_x) % grid_cells_y
                x = xmin_ + (cell_x + random.uniform(0.2, 0.8)) * (xmax_ - xmin_) / grid_cells_x
                y = ymin_ + (cell_y + random.uniform(0.2, 0.8)) * (ymax_ - ymin_) / grid_cells_y
            
            if is_valid_position(x, y, pts):
                pts.append((x, y))
                cell = get_grid_cell(x, y)
                if cell not in grid:
                    grid[cell] = []
                grid[cell].append((x, y))
            
            tries += 1
        
        if len(pts) < n:
            print(f"[large_forest] Placed {len(pts)}/{n} trees after {tries} tries")
        else:
            print(f"[large_forest] Successfully placed all {n} trees")
        
        return pts

    total = num_a + num_b
    pts = sample_points_improved(total, xmin_eff, xmax_eff, ymin_eff, ymax_eff, 
                                min_sep, max_sep, spawn_x, spawn_y)
    
    if len(pts) < total:
        print(f"[large_forest] Warning: Could only place {len(pts)}/{total} trees")
        print(f"[large_forest] Try reducing min_sep or increasing bounds")

    def inc(uri, name, x, y, z_, yaw):
        return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
    </include>"""

    # Create forest environment
    blocks = []
    blocks.append(inc("model://forest_env", "forest_env", 0, 0, 0, 0.0))
    
    # Place trees with some variation in height and rotation
    for i, (x, y) in enumerate(pts[:num_a]):
        height_variation = random.uniform(-0.1, 0.1)  # Small height variation
        blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z + height_variation, 
                         random.uniform(0.0, 2*math.pi)))
    
    for j, (x, y) in enumerate(pts[num_a:num_a+num_b]):
        height_variation = random.uniform(-0.1, 0.1)  # Small height variation
        blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z + height_variation, 
                         random.uniform(0.0, 2*math.pi)))

    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="large_forest_world">
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.5 0.3 1</ambient>
            <diffuse>0.3 0.5 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
{"".join(blocks)}
  </world>
</sdf>"""

    out_dir = os.path.join(os.path.expanduser('~'), '.ros')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'husky_large_forest.world.sdf')
    with open(out_path, 'w') as f:
        f.write(world_sdf)
    print(f"[large_forest] Wrote {out_path} with seed {seed}")
    print(f"[large_forest] Forest bounds: {xmin:.1f},{ymin:.1f} to {xmax:.1f},{ymax:.1f}")
    return out_path

# ------------------------
# Helpers
# ------------------------
def resolve_spawn_arg(context, name, lo, hi):
    val = LaunchConfiguration(name).perform(context)
    return str(random.uniform(lo, hi)) if val == 'RANDOM' else val

# ------------------------
# Setup phase
# ------------------------
def launch_setup(context, *args, **kwargs):
    # Resolve robot spawn - place in center for large forest
    spawn_x_val = resolve_spawn_arg(context, 'spawn_x', -2.0, 2.0)
    spawn_y_val = resolve_spawn_arg(context, 'spawn_y', -2.0, 2.0)
    spawn_yaw_val = resolve_spawn_arg(context, 'spawn_yaw', 0.0, 2*math.pi)

    # Make spawn available for world generation
    context.launch_configurations['resolved_spawn_x'] = spawn_x_val
    context.launch_configurations['resolved_spawn_y'] = spawn_y_val

    # Generate large forest world file
    world_path = generate_large_forest_world(context)

    # Start Gazebo with that world
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen'
    )

    # Spawn robot
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
    pkg_path = get_package_share_directory('forest_guard_sim')
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
    # ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))

    # Large Forest args - 4x larger area and 4x more trees
    large_forest_args = [
        # 4x more trees: 72 type A + 8 type B = 80 total (was 18+2=20)
        DeclareLaunchArgument('num_a', default_value='72'),
        DeclareLaunchArgument('num_b', default_value='8'),
        # 4x larger area: -20 to 20 = 40x40m (was -5 to 5 = 10x10m)
        DeclareLaunchArgument('xmin', default_value='-20.0'),
        DeclareLaunchArgument('xmax', default_value='20.0'),
        DeclareLaunchArgument('ymin', default_value='-20.0'),
        DeclareLaunchArgument('ymax', default_value='20.0'),
        # Slightly larger margin for bigger area
        DeclareLaunchArgument('margin', default_value='1.0'),
        # Slightly larger separation for more natural forest
        DeclareLaunchArgument('min_sep', default_value='1.0'),
        DeclareLaunchArgument('max_sep', default_value='4.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        # Fixed seed for reproducible static forest
        DeclareLaunchArgument('seed', default_value='42'),  # Fixed seed
    ]
    
    for arg in large_forest_args:
        ld.add_action(arg)

    # Husky spawn args - place near center for large forest
    ld.add_action(DeclareLaunchArgument('spawn_x', default_value='0.0'))  # Center
    ld.add_action(DeclareLaunchArgument('spawn_y', default_value='0.0'))  # Center
    ld.add_action(DeclareLaunchArgument('spawn_yaw', default_value='0.0'))

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

    # RViz (optional)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'forest.rviz'])],
    #     condition=IfCondition(LaunchConfiguration('rviz'))
    # )
    # ld.add_action(rviz_node)

    # Nav2 (optional) - important for large forest mapping
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_path, 'launch', 'navigation.launch.py'])]
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # Generate world + start Gazebo + spawn robot
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld