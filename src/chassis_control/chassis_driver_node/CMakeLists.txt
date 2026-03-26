cmake_minimum_required(VERSION 3.8)
project(chassis_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#指定头文件目录
# 添加头文件
include_directories(
  include
)
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include/chassis_driver_node
)

add_executable(chassis_driver_node src/chassis_driver_node.cpp)
ament_target_dependencies(chassis_driver_node rclcpp std_msgs  geometry_msgs)

target_include_directories(chassis_driver_node PRIVATE ${ZeroMQ_INCLUDE_DIRS})
target_link_libraries(chassis_driver_node ${ZeroMQ_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS
chassis_driver_node
  DESTINATION lib/${PROJECT_NAME}
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
