# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mobility_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mobility_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mobility_FOUND FALSE)
  elseif(NOT mobility_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mobility_FOUND FALSE)
  endif()
  return()
endif()
set(_mobility_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mobility_FIND_QUIETLY)
  message(STATUS "Found mobility: 0.0.0 (${mobility_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mobility' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT mobility_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mobility_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mobility_DIR}/${_extra}")
endforeach()
