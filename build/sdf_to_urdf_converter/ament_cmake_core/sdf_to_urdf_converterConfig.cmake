# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sdf_to_urdf_converter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sdf_to_urdf_converter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sdf_to_urdf_converter_FOUND FALSE)
  elseif(NOT sdf_to_urdf_converter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sdf_to_urdf_converter_FOUND FALSE)
  endif()
  return()
endif()
set(_sdf_to_urdf_converter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sdf_to_urdf_converter_FIND_QUIETLY)
  message(STATUS "Found sdf_to_urdf_converter: 0.0.0 (${sdf_to_urdf_converter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sdf_to_urdf_converter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sdf_to_urdf_converter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sdf_to_urdf_converter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sdf_to_urdf_converter_DIR}/${_extra}")
endforeach()
