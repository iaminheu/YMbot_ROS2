# Install script for directory: /home/ymzz/YMbot_ROS2/src/control/ymbot_d_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ymzz/YMbot_ROS2/install/ymbot_d_control")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control" TYPE EXECUTABLE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/keyboard_control_by_multiple_canables")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables"
         OLD_RPATH "/home/ymzz/YMbot_ROS2/src/control/ymbot_hardware_driver/lib_x86_64:/home/ymzz/YMbot_ROS2/install/ymbot_hardware_driver/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/keyboard_control_by_multiple_canables")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ymzz/YMbot_ROS2/build/ymbot_d_control/CMakeFiles/keyboard_control_by_multiple_canables.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control" TYPE EXECUTABLE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/modify_motors_pid")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid"
         OLD_RPATH "/home/ymzz/YMbot_ROS2/src/control/ymbot_hardware_driver/lib_x86_64:/home/ymzz/YMbot_ROS2/install/ymbot_hardware_driver/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/modify_motors_pid")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ymzz/YMbot_ROS2/build/ymbot_d_control/CMakeFiles/modify_motors_pid.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control" TYPE EXECUTABLE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/pose_tracking_left_arm")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm"
         OLD_RPATH "/home/ymzz/ws_moveit/install/moveit_servo/lib:/opt/ros/jazzy/lib:/home/ymzz/ws_moveit/install/moveit_ros_planning_interface/lib:/home/ymzz/ws_moveit/install/moveit_ros_move_group/lib:/home/ymzz/ws_moveit/install/moveit_ros_warehouse/lib:/home/ymzz/ws_moveit/install/moveit_ros_planning/lib:/home/ymzz/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/ymzz/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/ymzz/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_left_arm")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ymzz/YMbot_ROS2/build/ymbot_d_control/CMakeFiles/pose_tracking_left_arm.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control" TYPE EXECUTABLE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/pose_tracking_right_arm")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm"
         OLD_RPATH "/home/ymzz/ws_moveit/install/moveit_servo/lib:/opt/ros/jazzy/lib:/home/ymzz/ws_moveit/install/moveit_ros_planning_interface/lib:/home/ymzz/ws_moveit/install/moveit_ros_move_group/lib:/home/ymzz/ws_moveit/install/moveit_ros_warehouse/lib:/home/ymzz/ws_moveit/install/moveit_ros_planning/lib:/home/ymzz/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/ymzz/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/ymzz/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/pose_tracking_right_arm")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ymzz/YMbot_ROS2/build/ymbot_d_control/CMakeFiles/pose_tracking_right_arm.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control" TYPE EXECUTABLE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/position_tracking_neck")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/position_tracking_neck")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ymzz/YMbot_ROS2/build/ymbot_d_control/CMakeFiles/position_tracking_neck.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control" TYPE EXECUTABLE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/body_joint_tracking")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking"
         OLD_RPATH "/home/ymzz/ws_moveit/install/moveit_servo/lib:/home/ymzz/ws_moveit/install/moveit_ros_planning_interface/lib:/home/ymzz/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/jazzy/lib:/home/ymzz/ws_moveit/install/moveit_ros_warehouse/lib:/home/ymzz/ws_moveit/install/moveit_ros_planning/lib:/home/ymzz/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/ymzz/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/ymzz/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ymbot_d_control/body_joint_tracking")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ymzz/YMbot_ROS2/build/ymbot_d_control/CMakeFiles/body_joint_tracking.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE DIRECTORY FILES "/home/ymzz/YMbot_ROS2/src/control/ymbot_d_control/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE DIRECTORY FILES "/home/ymzz/YMbot_ROS2/src/control/ymbot_d_control/config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ymbot_d_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ymbot_d_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control/environment" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control/environment" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_index/share/ament_index/resource_index/packages/ymbot_d_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control/cmake" TYPE FILE FILES
    "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_core/ymbot_d_controlConfig.cmake"
    "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/ament_cmake_core/ymbot_d_controlConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ymbot_d_control" TYPE FILE FILES "/home/ymzz/YMbot_ROS2/src/control/ymbot_d_control/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ymzz/YMbot_ROS2/build/ymbot_d_control/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
