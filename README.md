Here’s a full `README.md` you can paste straight in, with added sections for:

* making the app easy to run on your machine, and
* editing `~/.bashrc` so ROS + the workspace + Ignition paths are always sourced.

````markdown
# ForestGuard – Handover: Code and Runbook

ForestGuard is a ROS 2 Humble-based simulation stack for a Husky-style UGV operating in a forest environment. It integrates LiDAR-based tree detection, Nav2 navigation, AMCL localisation, a custom Qt/PySide6 UI, and Xbox controller teleop.

This README documents how to install, run, and tune the final integrated code (branch: `main`).

---

## Code Access

GitHub repository:  
https://github.com/prollylole/ForestGuard.git  

Final integrated code lives on the `main` branch.

---

## Prerequisites

Assumed platform:

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Gazebo / Gazebo (ros-gz)

---

## Installation

### 1. System Dependencies

In a terminal:

```bash
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
````

### 2. Python Dependencies

```bash
pip install numpy opencv-python shapely matplotlib pyyaml trimesh PySide6
```

---

## Workspace Setup

Assume a ROS 2 workspace at `~/git/RS1`:

```bash
# Create workspace
mkdir -p ~/git/RS1/src
cd ~/git/RS1/src

# Clone repo
git clone https://github.com/prollylole/ForestGuard.git

cd ~/git/RS1
```

Install ROS dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

> If you are using a separate workspace folder called `main`, adjust the paths (e.g. `~/git/RS1/main` instead of `~/git/RS1`).

---

## Ignition Gazebo Environment Variables

Set the resource path so Ignition/Gazebo can find the worlds (update the path if your layout is different):

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/git/RS1/src/ForestGuard/forestguard_sim/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/git/RS1/src/ForestGuard/forestguard_sim/worlds
```

---

## Making the ForestGuard App Easy to Run

You can create a small shell script so the app “exists” on your device as a one-shot command.

1. Create a `bin` directory (if you don’t already have one):

```bash
mkdir -p $HOME/bin
```

2. Create a script for the demo:

```bash
nano $HOME/bin/forestguard_demo.sh
```

3. Paste this into the script (adjust paths if needed):

```bash
#!/usr/bin/env bash

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# ForestGuard workspace
# If you built in ~/git/RS1/main, change this to that install path.
if [ -f "$HOME/git/RS1/install/setup.bash" ]; then
  source "$HOME/git/RS1/install/setup.bash"
fi

# Ignition / Gazebo resources
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:$HOME/git/RS1/src/ForestGuard/forestguard_sim/worlds"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/git/RS1/src/ForestGuard/forestguard_sim/worlds"

# Launch the main simulation + UI + RViz
ros2 launch forestguard_sim JOHNAUTO2.launch.py \
  rviz:=true ui:=true teleop:=true amcl:=true slam:=false map:=true
```

4. Make it executable:

```bash
chmod +x $HOME/bin/forestguard_demo.sh
```

5. (Optional) Add `~/bin` to your PATH (see next section). Then you can run:

```bash
forestguard_demo.sh
```

from any terminal to start the app.

---

## Persisting Environment Setup in `~/.bashrc`

To avoid manually sourcing ROS and the workspace every time, you can add this snippet to the end of your `~/.bashrc`.

1. Open `~/.bashrc`:

```bash
nano ~/.bashrc
```

2. Add this block at the bottom (adjust paths to match your setup):

```bash
# ----- ROS 2 Humble -----
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# ----- ForestGuard workspace -----
# Change RS1 to RS1/main if that is where you built.
if [ -f "$HOME/git/RS1/install/setup.bash" ]; then
  source "$HOME/git/RS1/install/setup.bash"
fi

# ----- Ignition / Gazebo resources for ForestGuard -----
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:$HOME/git/RS1/src/ForestGuard/forestguard_sim/worlds"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/git/RS1/src/ForestGuard/forestguard_sim/worlds"

# ----- User scripts (e.g. forestguard_demo.sh) -----
export PATH="$HOME/bin:$PATH"
```

3. Reload your shell config:

```bash
source ~/.bashrc
```

From now on, every new terminal will:

* have ROS 2 Humble sourced,
* have the ForestGuard workspace sourced (if present),
* have the Ignition/Gazebo resource paths set, and
* have `$HOME/bin` in `PATH` so you can run `forestguard_demo.sh` directly.

---

## Optional: Data Logging (rosbag2)

```bash
sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
```

---

## Using a Joystick (Xbox Controller)

Check that the joystick is detected:

```bash
ls /dev/input/js*
```

Start the ROS 2 joystick node (in a sourced terminal):

```bash
ros2 run joy joy_node
```

The ForestGuard UI expects an Xbox-style controller with:

* Right bumper (RB): teleop gate (hold to authorise manual movement)
* Left stick: linear and angular velocity
* D-pad up/down: speed scaling
* Y button: camera feed toggle (normal vs HSV tuning)

---

## Main Launch Commands

Open a terminal (with ROS + workspace sourced, either manually or via `.bashrc`).

### 1. Simulation + UI + RViz

```bash
ros2 launch forestguard_sim JOHNAUTO2.launch.py \
  rviz:=true ui:=true teleop:=true amcl:=true slam:=false map:=true
```

This should open three windows:

1. **RViz** – live LiDAR scan, TF, costmaps, tree markers
2. **ForestGuard UI** – live camera feed, HSV tuning, LiDAR map, costmap, joystick + battery status
3. **Ignition/Gazebo** – simulated forest environment with robot model

### 2. Autonomy / Behaviour Layer

In a second sourced terminal:

```bash
ros2 launch forestguard_behaviour autonomy_bringup.launch.py \
  rviz:=true ui:=true teleop:=true amcl:=true slam:=false map:=true
```

This brings up Nav2, AMCL, and the autonomy nodes that use the LiDAR tree detections and mission logic.

---

## Running the Scenario (Xbox + UI)

1. Plug in the Xbox controller and ensure `/dev/input/js0` exists.
2. Start `joy_node` (see above).
3. With the UI running, **hold RB**:

   * The Teleop indicator in the UI should turn **green** while RB is held.
   * While green, the robot will respond to joystick commands.
4. Use the left stick to drive the robot in the Gazebo world.
5. Use D-pad up/down to adjust speed; the UI shows the current speed scaling.
6. Press `Y` to toggle between:

   * Standard camera feed
   * HSV tuning camera feed (for adjusting colour thresholds)

Known behaviour:

* Switching camera feeds with `Y` can have a small delay before the new feed appears.
* Teleop is only active while RB is held; if the indicator on the UI is not green, the robot will not move.

---

## Key Parameters

Below are the main parameters you may want to tune. Exact parameter files live under the relevant packages (e.g. `forestguard_localisation`, `forestguard_sim`, etc.).

### Perception (LiDAR Tree Detection)

| Parameter            | Purpose                                            | Example |
| -------------------- | -------------------------------------------------- | ------- |
| `min_sep_m`          | Minimum distance between tree detections (m)       | `3.0`   |
| `deadzone_frac`      | Fraction of LiDAR scan ignored near robot          | `0.5`   |
| `cluster_break_m`    | Distance threshold for LiDAR cluster splitting (m) | `0.20`  |
| `adaptive_break_k`   | Adaptive scaling factor for cluster breaking       | `2.0`   |
| `trunk_r_min_m`      | Minimum expected trunk radius (m)                  | `0.06`  |
| `trunk_r_max_m`      | Maximum expected trunk radius (m)                  | `0.50`  |
| `max_range_m`        | Maximum LiDAR detection range (m)                  | `12.0`  |
| `min_arc_deg`        | Minimum arc span for detection (degrees)           | `20.0`  |
| `max_cluster_span_m` | Maximum span of detected cluster (m)               | `1.2`   |
| `circularity_min`    | Minimum circularity threshold for trunk detection  | `0.35`  |
| `marker_scale_m`     | Size of visualisation markers                      | `0.35`  |
| `marker_lifetime_s`  | Marker lifetime in RViz (0 = infinite)             | `0.0`   |

### AMCL / Navigation

| Parameter             | Purpose                                            | Example |
| --------------------- | -------------------------------------------------- | ------- |
| `cov_threshold`       | Covariance threshold for “converged” localisation  | `0.10`  |
| `rotation_speed`      | Spin speed (rad/s) while waiting for convergence   | `0.50`  |
| `transform_tolerance` | TF time tolerance (s) for `map`↔`odom`↔`base_link` | `1.0`   |
| `update_min_a`        | Min rotation (rad) before filter update            | `0.20`  |
| `update_min_d`        | Min translation (m) before filter update           | `0.25`  |
| `max_particles`       | Max AMCL particles                                 | `2000`  |
| `min_particles`       | Min AMCL particles                                 | `500`   |

### Global / Local Costmap

| Parameter                             | Purpose                               | Example |
| ------------------------------------- | ------------------------------------- | ------- |
| `update_frequency`                    | Costmap update frequency (Hz)         | `1.0`   |
| `publish_frequency`                   | Costmap publish frequency (Hz)        | `1.0`   |
| `track_unknown_space` (global)        | Track unknown space                   | `True`  |
| `obstacle_layer.enabled`              | Enable obstacle layer                 | `True`  |
| `obstacle_layer.max_obstacle_height`  | Max obstacle height (m)               | `2.0`   |
| `inflation_layer.cost_scaling_factor` | Cost decay rate vs distance           | `3.0`   |
| `inflation_layer.inflation_radius`    | Inflation radius around obstacles (m) | `0.55`  |

### Xbox UI / Battery / Controller Model

| Parameter            | Purpose                        | Example         |
| -------------------- | ------------------------------ | --------------- |
| `voltage_full`       | Fully charged voltage (V)      | `12.0`          |
| `voltage_empty`      | Empty voltage (V)              | `5.0`           |
| `idle_drain_per_sec` | Idle drain rate (1/s)          | `1.0/600.0`     |
| `k_lin`              | Linear drain multiplier        | `3.0`           |
| `k_ang`              | Angular drain multiplier       | `1.5`           |
| `v_max`              | Max linear speed (m/s)         | `0.60`          |
| `W_max`              | Max angular speed (rad/s)      | `1.5`           |
| `initial_soc`        | Initial state of charge (0–1)  | `1.0`           |
| `lin_scale`          | UI joystick linear scaling     | `0.5`           |
| `ang_scale`          | UI joystick angular scaling    | `1.0`           |
| `fallback_cameras`   | Priority list of camera topics | `/camera/image` |

### Environment (Gazebo World)

| Parameter               | Purpose                      | Example                  |
| ----------------------- | ---------------------------- | ------------------------ |
| `gravity`               | Gravitational acceleration   | `0 0 -9.81`              |
| `magnetic_field`        | Environmental magnetic field | `6e-06 2.3e-05 -4.2e-05` |
| `max_step_size`         | Physics time step            | `0.001`                  |
| `real_time_update_rate` | Physics update rate (Hz)     | `1000`                   |
| `ambient`               | Ambient light colour         | `0.4 0.4 0.4 0.4`        |
| `background`            | Scene background colour      | `0.6 0.8 1.0 1.0`        |
| `sun`                   | Directional light direction  | `-0.5 0.5 -1`            |
| `Forest_env`            | Terrain model                | `model://forest_env`     |

---

## Notes / Known Issues

* Camera feed switching via the Xbox `Y` button may have a short delay.
* Teleop is only active while RB is held; if the indicator on the UI is not green, the robot will not move.
* If AMCL does not converge (high covariance), navigation goals may fail or behave erratically; check `cov_threshold`, initial pose, and sensor topics.

---

## Further Development

For further development and detailed tuning, see the parameter YAML files and launch files under the ForestGuard packages:

* `forestguard_sim`
* `forestguard_behaviour`
* `forestguard_localisation`

These contain the full configuration used in the final integrated system.

```
```
