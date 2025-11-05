#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
slam_demo_dir="$(dirname "$script_dir")"
mkdir -p "$slam_demo_dir/maps"
ros2 run nav2_map_server map_saver_cli -f "$slam_demo_dir/maps/office"
echo "Saved map to $slam_demo_dir/maps/office.(pgm|yaml)"
