cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(ymbot_d_sharedmemory_driver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

# 查找必要的库
find_package(YAML-CPP REQUIRED)

# 包含头文件目录
include_directories(
    ${YAML_CPP_INCLUDE_DIR}
    include
    # ${CMAKE_CURRENT_SOURCE_DIR}/config
)

# 设置需要链接的动态库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(ymbot_d_eumotor_interface 
    src/ymbot_d_eumotor_interface.cpp 
    src/ymbot_joint_eu.cpp)

add_executable(ymbot_d_eumotor_interface_test 
    src/ymbot_d_eumotor_interface_test.cpp)


# 链接库
target_link_libraries(ymbot_d_eumotor_interface
    ${YAML_CPP_LIBRARIES}
    libcontrolcan.so
    libeu_canable.so
    libeu_planet.so
    libicudata.so.60
    libicui18n.so.60
    libicuuc.so.60
    libnl-3.so.200
    libnl-route-3.so.200
    libcyhcs_log.so
)


# 可选：设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")    
