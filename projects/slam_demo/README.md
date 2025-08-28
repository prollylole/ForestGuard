# SLAM Demo (ROS 2 Humble + TurtleBot3 + slam_toolbox)

## Run (4 terminals)
T1:
  source /opt/ros/humble/setup.bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
T2:
  source /opt/ros/humble/setup.bash
  ros2 run rviz2 rviz2
  # In RViz: Add "Map" display → Topic: /map
T3:
  source /opt/ros/humble/setup.bash
  ros2 launch slam_toolbox online_async_launch.py
T4:
  source /opt/ros/humble/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Save map
  ros2 run nav2_map_server map_saver_cli -f $(pwd)/maps/office
## Webhook Test
