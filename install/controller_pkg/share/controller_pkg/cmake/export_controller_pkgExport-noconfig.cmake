#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "controller_pkg::quadcopter" for configuration ""
set_property(TARGET controller_pkg::quadcopter APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(controller_pkg::quadcopter PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/controller_pkg/quadcopter"
  )

list(APPEND _cmake_import_check_targets controller_pkg::quadcopter )
list(APPEND _cmake_import_check_files_for_controller_pkg::quadcopter "${_IMPORT_PREFIX}/lib/controller_pkg/quadcopter" )

# Import target "controller_pkg::laserprocessing" for configuration ""
set_property(TARGET controller_pkg::laserprocessing APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(controller_pkg::laserprocessing PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/controller_pkg/liblaserprocessing.so"
  IMPORTED_SONAME_NOCONFIG "liblaserprocessing.so"
  )

list(APPEND _cmake_import_check_targets controller_pkg::laserprocessing )
list(APPEND _cmake_import_check_files_for_controller_pkg::laserprocessing "${_IMPORT_PREFIX}/lib/controller_pkg/liblaserprocessing.so" )

# Import target "controller_pkg::vismarker" for configuration ""
set_property(TARGET controller_pkg::vismarker APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(controller_pkg::vismarker PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/controller_pkg/vismarker"
  )

list(APPEND _cmake_import_check_targets controller_pkg::vismarker )
list(APPEND _cmake_import_check_files_for_controller_pkg::vismarker "${_IMPORT_PREFIX}/lib/controller_pkg/vismarker" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
