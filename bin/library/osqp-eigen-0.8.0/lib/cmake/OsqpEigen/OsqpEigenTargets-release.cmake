#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OsqpEigen::OsqpEigen" for configuration "Release"
set_property(TARGET OsqpEigen::OsqpEigen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libOsqpEigen.so.0.8.0"
  IMPORTED_SONAME_RELEASE "libOsqpEigen.so.0.8.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS OsqpEigen::OsqpEigen )
list(APPEND _IMPORT_CHECK_FILES_FOR_OsqpEigen::OsqpEigen "${_IMPORT_PREFIX}/lib/libOsqpEigen.so.0.8.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
