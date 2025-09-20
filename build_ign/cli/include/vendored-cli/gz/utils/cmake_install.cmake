# Install script for directory: /home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/claudia/ForestGuard/utils_install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ignition/utils1/gz/utils/cli" TYPE FILE FILES
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/App.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/CLI.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Config.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/ConfigFwd.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Error.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Formatter.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/FormatterFwd.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Macros.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Option.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Split.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/StringTools.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Timer.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/TypeTools.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Validators.hpp"
    "/home/claudia/ForestGuard/src/ign-utils/cli/include/vendored-cli/gz/utils/cli/Version.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ignition/utils1/gz/utils/cli/.." TYPE FILE FILES "/home/claudia/ForestGuard/build_ign/cli/include/vendored-cli/gz/utils/cli.hh")
endif()

