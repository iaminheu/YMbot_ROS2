# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ymbot_d_ros2_controller_sharedmemory_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ymbot_d_ros2_controller_sharedmemory_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ymbot_d_ros2_controller_sharedmemory_FOUND FALSE)
  elseif(NOT ymbot_d_ros2_controller_sharedmemory_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ymbot_d_ros2_controller_sharedmemory_FOUND FALSE)
  endif()
  return()
endif()
set(_ymbot_d_ros2_controller_sharedmemory_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ymbot_d_ros2_controller_sharedmemory_FIND_QUIETLY)
  message(STATUS "Found ymbot_d_ros2_controller_sharedmemory: 0.0.0 (${ymbot_d_ros2_controller_sharedmemory_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ymbot_d_ros2_controller_sharedmemory' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ymbot_d_ros2_controller_sharedmemory_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ymbot_d_ros2_controller_sharedmemory_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${ymbot_d_ros2_controller_sharedmemory_DIR}/${_extra}")
endforeach()
