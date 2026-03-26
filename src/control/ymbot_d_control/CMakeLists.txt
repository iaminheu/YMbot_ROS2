cmake_minimum_required(VERSION 3.8)
project(ymbot_d_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(ymbot_hardware_driver REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)  




add_executable(keyboard_control_by_multiple_canables src/keyboard_control_by_multiple_canables.cpp)
target_include_directories(keyboard_control_by_multiple_canables PUBLIC include)
ament_target_dependencies(keyboard_control_by_multiple_canables ymbot_hardware_driver)


add_executable(modify_motors_pid src/modify_motors_pid.cpp)
target_include_directories(modify_motors_pid PUBLIC include)
ament_target_dependencies(modify_motors_pid ymbot_hardware_driver)


add_executable(pose_tracking_left_arm src/pose_tracking_left_arm.cpp)
target_include_directories(pose_tracking_left_arm PUBLIC include)
ament_target_dependencies(pose_tracking_left_arm moveit_msgs moveit_servo)


add_executable(pose_tracking_right_arm src/pose_tracking_right_arm.cpp)
target_include_directories(pose_tracking_right_arm PUBLIC include)
ament_target_dependencies(pose_tracking_right_arm moveit_msgs moveit_servo)


add_executable(position_tracking_neck src/position_tracking_neck.cpp)
target_include_directories(position_tracking_neck PUBLIC include)
ament_target_dependencies(position_tracking_neck rclcpp sensor_msgs)

add_executable(body_joint_tracking src/body_servo_node.cpp)
ament_target_dependencies(body_joint_tracking rclcpp std_srvs moveit_ros_planning moveit_msgs trajectory_msgs moveit_ros_planning_interface moveit_servo)





install(
  TARGETS
    keyboard_control_by_multiple_canables
    modify_motors_pid
    pose_tracking_left_arm
    pose_tracking_right_arm
    position_tracking_neck
    body_joint_tracking
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 