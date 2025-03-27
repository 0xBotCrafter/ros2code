cmake_minimum_required(VERSION 3.10)
project(chapt2_1_first_node_cpp)

# 1.查找依赖的包
find_package(ament_cmake REQUIRED) # 构建工具
find_package(rclcpp REQUIRED) # 客户端库

# 2.生成可执行文件
add_executable(first_node_cpp first_node_cpp.cpp)

# 3.添加依赖到可执行文件
ament_target_dependencies(first_node_cpp rclcpp)
