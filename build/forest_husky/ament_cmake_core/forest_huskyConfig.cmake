# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_forest_husky_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED forest_husky_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(forest_husky_FOUND FALSE)
  elseif(NOT forest_husky_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(forest_husky_FOUND FALSE)
  endif()
  return()
endif()
set(_forest_husky_CONFIG_INCLUDED TRUE)

# output package information
if(NOT forest_husky_FIND_QUIETLY)
  message(STATUS "Found forest_husky: 1.0.3 (${forest_husky_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'forest_husky' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${forest_husky_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(forest_husky_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${forest_husky_DIR}/${_extra}")
endforeach()
