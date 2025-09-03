# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robo1_haje_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robo1_haje_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robo1_haje_FOUND FALSE)
  elseif(NOT robo1_haje_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robo1_haje_FOUND FALSE)
  endif()
  return()
endif()
set(_robo1_haje_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robo1_haje_FIND_QUIETLY)
  message(STATUS "Found robo1_haje: 0.0.1 (${robo1_haje_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robo1_haje' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robo1_haje_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robo1_haje_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robo1_haje_DIR}/${_extra}")
endforeach()
