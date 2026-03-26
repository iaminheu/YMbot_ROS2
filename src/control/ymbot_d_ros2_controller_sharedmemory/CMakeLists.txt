cmake_minimum_required(VERSION 3.16)
project(ymbot_d_ros2_controller_sharedmemory LANGUAGES CXX)

# 添加编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find_package(ymbot_hardware_driver REQUIRED)

# 添加库
add_library(ymbot_d_ros2_controller_sharedmemory SHARED
  src/ymbot_d_hardware_interface.cpp
)

# 设置目标编译特性
target_compile_features(ymbot_d_ros2_controller_sharedmemory PUBLIC cxx_std_17)

# 设置包含目录
target_include_directories(ymbot_d_ros2_controller_sharedmemory PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ymbot_d_ros2_controller_sharedmemory>
)

# 设置目标依赖项
ament_target_dependencies(ymbot_d_ros2_controller_sharedmemory PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # ymbot_hardware_driver
)

# 设置编译定义
target_compile_definitions(ymbot_d_ros2_controller_sharedmemory PRIVATE "YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_BUILDING_DLL")

# 导出插件描述文件
pluginlib_export_plugin_description_file(hardware_interface ymbot_d_ros2_controller_sharedmemory.xml)

# 安装目标
install(
  TARGETS ymbot_d_ros2_controller_sharedmemory
  EXPORT export_ymbot_d_ros2_controller_sharedmemory
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装包含目录
install(
  DIRECTORY include/
  DESTINATION include/ymbot_d_ros2_controller_sharedmemory
)

# 导出依赖项、包含目录和库
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # ymbot_hardware_driver
)
ament_export_include_directories(include)
ament_export_libraries(ymbot_d_ros2_controller_sharedmemory)

# 标记为 ROS 2 包
ament_package()
