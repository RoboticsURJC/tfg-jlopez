# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pibotj_urdf_complete_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pibotj_urdf_complete_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pibotj_urdf_complete_FOUND FALSE)
  elseif(NOT pibotj_urdf_complete_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pibotj_urdf_complete_FOUND FALSE)
  endif()
  return()
endif()
set(_pibotj_urdf_complete_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pibotj_urdf_complete_FIND_QUIETLY)
  message(STATUS "Found pibotj_urdf_complete: 0.0.0 (${pibotj_urdf_complete_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pibotj_urdf_complete' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pibotj_urdf_complete_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pibotj_urdf_complete_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pibotj_urdf_complete_DIR}/${_extra}")
endforeach()
