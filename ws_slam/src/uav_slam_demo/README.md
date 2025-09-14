
# UAV 3D SLAM Demo (Gazebo → RTAB-Map → RViz) — ROS 2 Humble

This is a no‑hardware demo that shows **live 3D SLAM** (point cloud map + trajectory) with a simulated UAV carrying an **RGB‑D camera** and **IMU**.

## 0) Install (once)
```bash
sudo apt update
sudo apt install -y   ros-humble-ros-gz   ros-humble-rtabmap-ros   ros-humble-imu-filter-madgwick   ros-humble-image-pipeline   ros-humble-pcl-ros   ros-humble-teleop-twist-keyboard
```

## 1) Source your ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
```

## 2) Put this folder in your workspace and build (so RViz/launch finds package share)
```bash
# If you don't have a colcon workspace yet:
mkdir -p ~/ws_slam/src
# Copy the 'uav_slam_demo' folder into ~/ws_slam/src
cp -r uav_slam_demo ~/ws_slam/src/

cd ~/ws_slam
colcon build
source install/setup.bash
```

## 3) Run the demo (one command)
```bash
ros2 launch uav_slam_demo slam_demo.launch.py
```
This will:
- start **Gazebo** with a small indoor room and a UAV,
- bridge topics to ROS 2,
- run an **IMU filter**,
- start **RTAB‑Map** (RGB‑D + IMU + loop closures),
- open **RViz** (already configured).

## 4) Move the UAV (second terminal)
```bash
source ~/ws_slam/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/uav/cmd_vel
```
Use WASD to strafe/forward, R/F up/down, Q/E yaw. Move slowly at first.

## 5) What you should see
- In **RViz** (fixed frame: `map`): a growing **3D point cloud** (`/rtabmap/cloud_map`), pose (`/odom`), and **trajectory** (`/rtabmap/trajectory`).

## Notes / Troubleshooting
- If the map "melts": slow down motion; keep camera update rate ~15 Hz (already set).
- If RViz shows no data: ensure the launch terminal shows bridges up and that `/odom` is publishing.
- Everything uses **sim time** via `/clock` automatically.
- The static TF `base_link→camera_link` matches the SDF; if you move the camera in SDF, update the TF in `slam_demo.launch.py` too.
