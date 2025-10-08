cmake_minimum_required(VERSION 3.8)
project(husky_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(gtest_vendor REQUIRED)

# For pfms_types
# find_package(pfms2ros 3.0.7 REQUIRED)

# The library we are developing will be called laserprocessing
add_library(laserprocessing src/laserprocessing.cpp)
ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

## This is the executable
# add "quadcopter" node 
add_executable(husky 
  src/main_husky.cpp 
  src/controller_husky.cpp 
  src/husky.cpp)

ament_target_dependencies(husky 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  std_srvs
  visualization_msgs
)
  
target_link_libraries(husky laserprocessing)

target_include_directories(husky PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(laserprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# add "vismarker" node
add_executable(vismarker 
  src/main_vismarker.cpp 
  src/vis_marker.cpp)

ament_target_dependencies(vismarker
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  )
# (No need to link laserprocessing here, this node only subscribes
#  to PoseArray and publishes MarkerArray.)

target_include_directories(vismarker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  husky
  laserprocessing
  vismarker
  DESTINATION lib/${PROJECT_NAME} # this will install the executables and library to install/controller_pkg/lib/controller_pkg
  )

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY data
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)

  # ament_add_ros_isolated_gtest(test_laserprocessing test/utest.cpp)
  # target_link_libraries(test_laserprocessing laserprocessing)
  # ament_target_dependencies(test_laserprocessing rclcpp sensor_msgs nav_msgs tf2 tf2_geometry_msgs rosbag2_cpp)

endif()

ament_package()