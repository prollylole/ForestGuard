cmake_minimum_required(VERSION 3.8)
project(my_drone_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_plugins REQUIRED)

# Add executable for drone controller
add_executable(drone_controller src/drone_controller.cpp)

# Link dependencies
ament_target_dependencies(drone_controller rclcpp)

# Install the executable
install(TARGETS drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install setup script
install(PROGRAMS
  setup_gazebo.sh
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install urdf files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
ament_package()
