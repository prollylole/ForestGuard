#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
