# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_adore_global_planner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED adore_global_planner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(adore_global_planner_FOUND FALSE)
  elseif(NOT adore_global_planner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(adore_global_planner_FOUND FALSE)
  endif()
  return()
endif()
set(_adore_global_planner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT adore_global_planner_FIND_QUIETLY)
  message(STATUS "Found adore_global_planner: 0.1.0 (${adore_global_planner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'adore_global_planner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${adore_global_planner_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(adore_global_planner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${adore_global_planner_DIR}/${_extra}")
endforeach()
