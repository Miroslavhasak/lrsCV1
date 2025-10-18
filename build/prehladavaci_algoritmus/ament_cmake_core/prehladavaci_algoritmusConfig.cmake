# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_prehladavaci_algoritmus_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED prehladavaci_algoritmus_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(prehladavaci_algoritmus_FOUND FALSE)
  elseif(NOT prehladavaci_algoritmus_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(prehladavaci_algoritmus_FOUND FALSE)
  endif()
  return()
endif()
set(_prehladavaci_algoritmus_CONFIG_INCLUDED TRUE)

# output package information
if(NOT prehladavaci_algoritmus_FIND_QUIETLY)
  message(STATUS "Found prehladavaci_algoritmus: 0.0.1 (${prehladavaci_algoritmus_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'prehladavaci_algoritmus' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${prehladavaci_algoritmus_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(prehladavaci_algoritmus_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${prehladavaci_algoritmus_DIR}/${_extra}")
endforeach()
