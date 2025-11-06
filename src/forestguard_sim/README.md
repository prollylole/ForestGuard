# John

  Bringup for *John*. Launches a Husky robot in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

* Do this
  ```bash
  cd ForestGuard/src
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
  ```
* Source workspace (if you add this to your ~/.bashrc, then you don't need to do this each time)
  ```bash
  source ~/ForestGuard/src/install/setup.bash
  ```
* You can choose to add it to bashrc too

* Then do this
  ```bash
  ros2 launch john john.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
  ```
* When launching with rviz, you can send a waypoint to the robot by clicking the "2D Goal pose" and then a location in the map. The robot is navigating using the nav2 package. If it gets stuck, you can try the buttons in the Navigation 2 panel in the top right of RVIZ.

* You can also drive the robot using keyboard teleoperation by running the following in a separate terminal, then use the keys listed in the instructions to move the robot:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```