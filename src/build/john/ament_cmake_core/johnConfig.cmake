# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_john_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED john_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(john_FOUND FALSE)
  elseif(NOT john_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(john_FOUND FALSE)
  endif()
  return()
endif()
set(_john_CONFIG_INCLUDED TRUE)

# output package information
if(NOT john_FIND_QUIETLY)
  message(STATUS "Found john: 1.0.3 (${john_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'john' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${john_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(john_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${john_DIR}/${_extra}")
endforeach()
