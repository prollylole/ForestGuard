cmake_minimum_required(VERSION 3.5)
project(sdf_to_urdf_converter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf_parser_plugin REQUIRED)  
find_package(pluginlib REQUIRED) 

# Use pkg-config for tinyxml2
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

# Try to find sdformat with possible version-specific names
find_package(sdformat14 QUIET)
if(NOT sdformat14_FOUND)
  find_package(sdformat12 QUIET)
  if(NOT sdformat12_FOUND)
    find_package(sdformat REQUIRED)
  endif()
endif()

# Add include directory
include_directories(
  include
  ${urdf_parser_plugin_INCLUDE_DIRS}  
)

# Add library
add_library(sdformat_urdf SHARED
  src/sdformat_urdf.cpp
  src/sdformat_urdf_plugin.cpp
)

target_include_directories(sdformat_urdf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TINYXML2_INCLUDE_DIRS}
  ${SDFORMAT_INCLUDE_DIRS}
  ${urdf_parser_plugin_INCLUDE_DIRS} 
)

if(sdformat14_FOUND)
  target_include_directories(sdformat_urdf PUBLIC ${sdformat14_INCLUDE_DIRS})
  target_link_libraries(sdformat_urdf ${sdformat14_LIBRARIES})
elseif(sdformat12_FOUND)
  target_include_directories(sdformat_urdf PUBLIC ${sdformat12_INCLUDE_DIRS})
  target_link_libraries(sdformat_urdf ${sdformat12_LIBRARIES})
else()
  target_include_directories(sdformat_urdf PUBLIC ${sdformat_INCLUDE_DIRS})
  target_link_libraries(sdformat_urdf ${sdformat_LIBRARIES})
endif()

ament_target_dependencies(sdformat_urdf
  rcutils
  urdfdom
  urdf_parser_plugin
  pluginlib
)

target_link_libraries(sdformat_urdf
  ${TINYXML2_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
)

# Add executable to convert SDF to URDF
add_executable(convert_sdf_to_urdf src/convert_sdf_to_urdf.cpp)
target_include_directories(convert_sdf_to_urdf PUBLIC
  ${TINYXML2_INCLUDE_DIRS}
  ${SDFORMAT_INCLUDE_DIRS}
  ${urdf_parser_plugin_INCLUDE_DIRS}
)
target_link_libraries(convert_sdf_to_urdf
  sdformat_urdf
  ${TINYXML2_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
)

if(sdformat14_FOUND)
  target_include_directories(convert_sdf_to_urdf PUBLIC ${sdformat14_INCLUDE_DIRS})
  target_link_libraries(convert_sdf_to_urdf ${sdformat14_LIBRARIES})
elseif(sdformat12_FOUND)
  target_include_directories(convert_sdf_to_urdf PUBLIC ${sdformat12_INCLUDE_DIRS})
  target_link_libraries(convert_sdf_to_urdf ${sdformat12_LIBRARIES})
else()
  target_include_directories(convert_sdf_to_urdf PUBLIC ${sdformat_INCLUDE_DIRS})
  target_link_libraries(convert_sdf_to_urdf ${sdformat_LIBRARIES})
endif()

ament_target_dependencies(convert_sdf_to_urdf
  rcutils
  urdfdom
  urdf_parser_plugin
  pluginlib
)

# Install executable
install(TARGETS
  sdformat_urdf
  convert_sdf_to_urdf
  DESTINATION lib/${PROJECT_NAME}
)

# Install the header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()