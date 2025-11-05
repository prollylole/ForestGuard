cmake_minimum_required(VERSION 3.5)
project(uav_slam_demo)

find_package(ament_cmake REQUIRED)

# Install resources so FindPackageShare can locate them
install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
