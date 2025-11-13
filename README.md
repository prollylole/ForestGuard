# FORESTGUARD – HANDOVER: CODE AND RUNBOOK

ForestGuard is a ROS 2 Humble-based simulation stack for a Husky-style UGV operating in a forest environment. It integrates LiDAR-based tree detection, Nav2 navigation, AMCL localisation, a custom Qt/PySide6 UI, and Xbox controller teleop.

This README documents how to install, run, and tune the final integrated code (branch: john_branch).

---

## CODE ACCESS

GitHub repository:
[https://github.com/prollylole/ForestGuard.git](https://github.com/prollylole/ForestGuard.git)

Final integrated code lives on the `main` branch.

---

## PREREQUISITES

Assumed platform:

* Ubuntu 22.04
* ROS 2 Humble
* Ignition Gazebo / Gazebo (ros-gz)

---

## INSTALLATION

1. System Dependencies

---

In a terminal:

sudo apt update

# Colcon / tooling

sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool

# Nav2 / teleop / joystick

sudo apt install ros-humble-nav2-bringup ros-humble-xacro ros-humble-joy ros-humble-teleop-twist-joy

# Robot model / localisation

sudo apt install ros-humble-robot-state-publisher ros-humble-robot-localization

# Gazebo bridge + sim

sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Visualisation

sudo apt install ros-humble-rviz2

2. Python Dependencies

---

pip install numpy opencv-python shapely matplotlib pyyaml trimesh PySide6

---

## WORKSPACE SETUP

Assume a ROS 2 workspace at:  ~/git/RS1

Create the workspace and clone the repo:

mkdir -p ~/git/RS1/src
cd ~/git/RS1/src

git clone [https://github.com/prollylole/ForestGuard.git](https://github.com/prollylole/ForestGuard.git)

cd ~/git/RS1

Install ROS dependencies:

rosdep update
rosdep install --from-paths src --ignore-src -r -y

Build:

colcon build --symlink-install

Source the workspace:

source install/setup.bash

Note: If you are using a separate workspace folder called `john_branch`, adjust the paths accordingly (for example: `~/git/RS1/john_branch`).

---

## IGNITION GAZEBO ENVIRONMENT VARIABLES

Set the resource path so Ignition/Gazebo can find the worlds:

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/git/RS1/src/ForestGuard/forestguard_sim/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/git/RS1/src/ForestGuard/forestguard_sim/worlds

You can add these lines to your `~/.bashrc` so they persist.

---

## OPTIONAL: DATA LOGGING (ROSBAG2)

sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins

---

## USING A JOYSTICK (XBOX CONTROLLER)

Check that the joystick is detected:

ls /dev/input/js*

Start the ROS 2 joystick node (in a sourced terminal):

ros2 run joy joy_node

The ForestGuard UI expects an Xbox-style controller with:

* Right bumper (RB): teleop gate (hold to authorise manual movement).
* Left stick: linear and angular velocity.
* D-pad up/down: speed scaling.
* Y button: camera feed toggle (normal vs HSV tuning).

---

## MAIN LAUNCH COMMANDS

Open a terminal and source ROS:

source /opt/ros/humble/setup.bash
source ~/git/RS1/install/setup.bash

# or:  source ~/git/RS1/john_branch/install/setup.bash   (if using that workspace)

1. Simulation + UI + RViz

---

ros2 launch forestguard_sim JOHNAUTO2.launch.py 
rviz:=true ui:=true teleop:=true amcl:=true slam:=false map:=true

This should open three windows:

1. RViz – live LiDAR scan, TF, costmaps, tree markers.
2. ForestGuard UI – live camera feed, HSV tuning tab, LiDAR visualisation, costmap, joystick + battery status.
3. Ignition/Gazebo – simulated forest environment with robot model.

2) Autonomy / Behaviour Layer

---

In a second sourced terminal:

ros2 launch forestguard_behaviour autonomy_bringup.launch.py 
rviz:=true ui:=true teleop:=true amcl:=true slam:=false map:=true

This brings up Nav2, AMCL, and the autonomy nodes that use the LiDAR tree detections and mission logic.

---

## RUNNING THE SCENARIO (XBOX + UI)

1. Plug in the Xbox controller and ensure `/dev/input/js0` exists.
2. Start `joy_node` (see above).
3. With the UI running, hold RB:

   * The Teleop indicator in the UI should turn green while RB is held.
   * While green, the robot will respond to joystick commands.
4. Use the left stick to drive the robot in the Gazebo world.
5. Use D-pad up/down to adjust speed; the UI shows the current speed scaling.
6. Press Y to toggle between:

   * Standard camera feed.
   * HSV tuning camera feed (for adjusting colour thresholds).

Known behaviour:

* Switching camera feeds with Y can have a small delay before the new feed appears.
* Teleop is only active while RB is held; if the indicator on the UI is not green, the robot will not move.

---

## KEY PARAMETERS

Below are the main parameters you may want to tune. Exact parameter files live under the relevant packages (e.g. forestguard_localisation, forestguard_sim, etc.).

## PERCEPTION (LIDAR TREE DETECTION)

min_sep_m
Minimum distance between tree detections (m)
Example: 3.0

deadzone_frac
Fraction of LiDAR scan ignored near robot
Example: 0.5

cluster_break_m
Distance threshold for LiDAR cluster splitting (m)
Example: 0.20

adaptive_break_k
Adaptive scaling factor for cluster breaking
Example: 2.0

trunk_r_min_m
Minimum expected trunk radius (m)
Example: 0.06

trunk_r_max_m
Maximum expected trunk radius (m)
Example: 0.50

max_range_m
Maximum LiDAR detection range (m)
Example: 12.0

min_arc_deg
Minimum arc span for detection (degrees)
Example: 20.0

max_cluster_span_m
Maximum span of detected cluster (m)
Example: 1.2

circularity_min
Minimum circularity threshold for trunk detection
Example: 0.35

marker_scale_m
Size of visualisation markers
Example: 0.35

marker_lifetime_s
Marker lifetime in RViz (0 = infinite)
Example: 0.0

## AMCL / NAVIGATION

cov_threshold
Mean covariance threshold below which localisation is considered “converged”
Example: 0.10

rotation_speed
Angular speed (rad/s) for spin-in-place while waiting for AMCL
Example: 0.50

transform_tolerance
TF time tolerance (s) between map, odom, base_link
Example: 1.0

update_min_a
Minimum rotation (rad) before filter update
Example: 0.20

update_min_d
Minimum translation (m) before filter update
Example: 0.25

max_particles
Maximum number of particles in AMCL filter
Example: 2000

min_particles
Minimum number of particles in AMCL filter
Example: 500

## GLOBAL / LOCAL COSTMAP

update_frequency
Costmap update frequency (Hz)
Example: 1.0

publish_frequency
Costmap publish frequency (Hz)
Example: 1.0

track_unknown_space (global)
Whether to track unknown space
Example: True

obstacle_layer.enabled
Enable obstacle layer from sensor data
Example: True

obstacle_layer.max_obstacle_height
Max obstacle height (m)
Example: 2.0

inflation_layer.cost_scaling_factor
How quickly cost decays with distance
Example: 3.0

inflation_layer.inflation_radius
Inflation radius (m) around obstacles
Example: 0.55

## XBOX UI / BATTERY / CONTROLLER MODEL

voltage_full
Fully charged battery voltage (V)
Example: 12.0

voltage_empty
Empty battery voltage (V)
Example: 5.0

idle_drain_per_sec
Drain rate when idle (1/s)
Example: 1.0 / 600.0

k_lin
Linear speed drain multiplier
Example: 3.0

k_ang
Angular speed drain multiplier
Example: 1.5

v_max
Maximum linear speed (m/s)
Example: 0.60

W_max
Maximum angular speed (rad/s)
Example: 1.5

initial_soc
Initial state of charge (0–1)
Example: 1.0

lin_scale
UI joystick linear scaling
Example: 0.5

ang_scale
UI joystick angular scaling
Example: 1.0

fallback_cameras
Priority list of fallback camera topics
Example: /camera/image

## ENVIRONMENT (GAZEBO WORLD)

gravity
Gravitational acceleration (m/s^2)
Example: 0 0 -9.81

magnetic_field
Environmental magnetic field
Example: 6e-06 2.3e-05 -4.2e-05

max_step_size
Physics engine time step
Example: 0.001

real_time_update_rate
Physics update frequency (Hz)
Example: 1000

ambient
Ambient light colour
Example: 0.4 0.4 0.4 0.4

background
Scene background colour
Example: 0.6 0.8 1.0 1.0

sun
Directional light config (direction)
Example: -0.5 0.5 -1

Forest_env
Terrain model included
Example: model://forest_env

---

## NOTES / KNOWN ISSUES

* Camera feed switching via the Xbox Y button may have a short delay.
* Teleop is only active while RB is held; if the indicator on the UI is not green, the robot will not move.
* If AMCL does not converge (high covariance), navigation goals may fail or behave erratically; check cov_threshold, initial pose, and sensor topics.

---

## FURTHER DEVELOPMENT

For further development and detailed tuning, see the parameter YAML files and launch files under the ForestGuard packages:

* forestguard_sim
* forestguard_behaviour
* forestguard_localisation

These contain the full configuration used in the final integrated system.
