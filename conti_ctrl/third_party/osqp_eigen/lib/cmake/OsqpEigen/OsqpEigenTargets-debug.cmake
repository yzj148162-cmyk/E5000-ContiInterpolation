#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OsqpEigen::OsqpEigen" for configuration "Debug"
set_property(TARGET OsqpEigen::OsqpEigen APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/OsqpEigend.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/OsqpEigend.dll"
  )

list(APPEND _cmake_import_check_targets OsqpEigen::OsqpEigen )
list(APPEND _cmake_import_check_files_for_OsqpEigen::OsqpEigen "${_IMPORT_PREFIX}/lib/OsqpEigend.lib" "${_IMPORT_PREFIX}/bin/OsqpEigend.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
