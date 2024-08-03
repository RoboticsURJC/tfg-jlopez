# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pibotj_pothole_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pibotj_pothole_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pibotj_pothole_FOUND FALSE)
  elseif(NOT pibotj_pothole_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pibotj_pothole_FOUND FALSE)
  endif()
  return()
endif()
set(_pibotj_pothole_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pibotj_pothole_FIND_QUIETLY)
  message(STATUS "Found pibotj_pothole: 0.0.0 (${pibotj_pothole_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pibotj_pothole' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pibotj_pothole_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pibotj_pothole_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pibotj_pothole_DIR}/${_extra}")
endforeach()
