# from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable, OpaqueFunction, ExecuteProcess, DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from ament_index_python.packages import get_package_share_directory
# import os, math, random

# def _gen_and_run(context, *args, **kwargs):
#     # Read CLI args
#     num_a  = int(LaunchConfiguration('num_a').perform(context))   # count of tree1
#     num_b  = int(LaunchConfiguration('num_b').perform(context))   # count of tree2
#     xmin   = float(LaunchConfiguration('xmin').perform(context))
#     xmax   = float(LaunchConfiguration('xmax').perform(context))
#     ymin   = float(LaunchConfiguration('ymin').perform(context))
#     ymax   = float(LaunchConfiguration('ymax').perform(context))
#     min_sep = float(LaunchConfiguration('min_sep').perform(context))
#     max_sep = float(LaunchConfiguration('max_sep').perform(context))
#     margin  = float(LaunchConfiguration('margin').perform(context))
#     z      = float(LaunchConfiguration('z').perform(context))
#     seed   = int(LaunchConfiguration('seed').perform(context))
#     random.seed(seed)

#     # Guardrail: ensure feasible constraints
#     if max_sep <= min_sep:
#         print(f"[forest_random] max_sep ({max_sep}) <= min_sep ({min_sep}); nudging max_sep slightly up.")
#         max_sep = min_sep + 1e-6

#     # Effective bounds (keep-off margin from edges)
#     xmin_eff = xmin + margin
#     xmax_eff = xmax - margin
#     ymin_eff = ymin + margin
#     ymax_eff = ymax - margin

#     # Sampler with BOTH min and max separation (clusters)
#     def sample_points(n, xmin_, xmax_, ymin_, ymax_, min_s, max_s, seed_pts=1, max_tries=40000):
#         pts, tries = [], 0
#         min_r2 = min_s * min_s
#         max_r2 = max_s * max_s
#         while len(pts) < n and tries < max_tries:
#             x = random.uniform(xmin_, xmax_)
#             y = random.uniform(ymin_, ymax_)
#             # Enforce min distance from ALL placed points
#             if not all((x - xi)**2 + (y - yi)**2 >= min_r2 for xi, yi in pts):
#                 tries += 1
#                 continue
#             # Allow a seed to start the first cluster
#             if len(pts) < seed_pts:
#                 pts.append((x, y))
#                 continue
#             # Every subsequent point must be within max_sep of at least one existing point
#             if any((x - xi)**2 + (y - yi)**2 <= max_r2 for xi, yi in pts):
#                 pts.append((x, y))
#             tries += 1
#         return pts

#     total = num_a + num_b
#     pts = sample_points(total, xmin_eff, xmax_eff, ymin_eff, ymax_eff, min_sep, max_sep, seed_pts=1)
#     if len(pts) < total:
#         print(f"[forest_random] placed {len(pts)}/{total} trees; relax min_sep/increase max_sep or enlarge bounds.")

#     def inc(uri, name, x, y, z_, yaw):
#         return f"""
#     <include>
#       <uri>{uri}</uri>
#       <name>{name}</name>
#       <pose>{x:.3f} {y:.3f} {z_:.3f} 0 0 {yaw:.6f}</pose>
#     </include>"""

#     blocks = []
#     # Ground environment
#     blocks.append(inc("model://forest_env", "forest_env", 0, 0, 0, 0.0))
#     # First num_a -> tree1
#     for i, (x, y) in enumerate(pts[:num_a]):
#         blocks.append(inc("model://tree1", f"tree1_{i}", x, y, z, random.uniform(0.0, 2*math.pi)))
#     # Remaining -> tree2
#     for j, (x, y) in enumerate(pts[num_a:num_a+num_b]):
#         blocks.append(inc("model://tree2", f"tree2_{j}", x, y, z, random.uniform(0.0, 2*math.pi)))

#     world_sdf = f"""<?xml version="1.0" ?>
# <sdf version="1.9">
#   <world name="forest_world">
#     <gravity>0 0 -9.81</gravity>
#     <light type="directional" name="sun">
#       <cast_shadows>true</cast_shadows>
#       <pose>0 0 10 0 0 0</pose>
#       <diffuse>1 1 1 1</diffuse>
#       <specular>0.2 0.2 0.2 1</specular>
#       <direction>-0.5 0.5 -1</direction>
#     </light>
# {"".join(blocks)}
#   </world>
# </sdf>
# """
#     out_dir = os.path.join(os.path.expanduser('~'), '.ros')
#     os.makedirs(out_dir, exist_ok=True)
#     out_path = os.path.join(out_dir, 'forest_random.world.sdf')
#     with open(out_path, 'w') as f:
#         f.write(world_sdf)
#     print(f"[forest_random] wrote {out_path}")

#     return [ExecuteProcess(cmd=['ign', 'gazebo', '-r', out_path], output='screen')]

# def generate_launch_description():
#     pkg_share = get_package_share_directory('forest_guard_sim')
#     models = PathJoinSubstitution([pkg_share, 'models'])
#     worlds = PathJoinSubstitution([pkg_share, 'worlds'])

#     return LaunchDescription([
#         # Tunables (you can override on CLI)
#         DeclareLaunchArgument('num_a',   default_value='18'),
#         DeclareLaunchArgument('num_b',   default_value='2'),
#         DeclareLaunchArgument('xmin',    default_value='-5.0'),
#         DeclareLaunchArgument('xmax',    default_value='5.0'),
#         DeclareLaunchArgument('ymin',    default_value='-5.0'),
#         DeclareLaunchArgument('ymax',    default_value='5.0'),
#         DeclareLaunchArgument('margin',  default_value='0.5'),
#         DeclareLaunchArgument('min_sep', default_value='0.8'),
#         DeclareLaunchArgument('max_sep', default_value='3'),
#         DeclareLaunchArgument('z',       default_value='0.0'),
#         DeclareLaunchArgument('seed',    default_value='42'),

#         # Make model:// URIs resolve
#         SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', [models, ':', worlds]),
#         SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [models, ':', worlds]),

#         # Generate world + run Ignition
#         OpaqueFunction(function=_gen_and_run),
#     ])
