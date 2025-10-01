#!/bin/bash

# Get the ROS 2 installation path
ROS2_DISTRO=$(printenv ROS_DISTRO)
ROS2_PATH="/opt/ros/$ROS2_DISTRO"

# Set Gazebo plugin path to include ROS plugins
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$ROS2_PATH/lib

# Set model path if needed
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix my_drone_pkg)/share/my_drone_pkg/models

echo "Gazebo environment configured for ROS 2 $ROS2_DISTRO"