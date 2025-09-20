#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ignition-utils1::ignition-utils1" for configuration "RelWithDebInfo"
set_property(TARGET ignition-utils1::ignition-utils1 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ignition-utils1::ignition-utils1 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libignition-utils1.so.1.5.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libignition-utils1.so.1"
  )

list(APPEND _cmake_import_check_targets ignition-utils1::ignition-utils1 )
list(APPEND _cmake_import_check_files_for_ignition-utils1::ignition-utils1 "${_IMPORT_PREFIX}/lib/libignition-utils1.so.1.5.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
