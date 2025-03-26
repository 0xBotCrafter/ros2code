cmake_minimum_required(VERSION 3.10)
project(chapt2_1_first_node_cpp)

# Add executable
add_executable(first_node_cpp first_node_cpp.cpp)

find_package(ament_cmake REQUIRED) # find ament_cmake package
find_package(rclcpp REQUIRED) # find rclcpp package

ament_target_dependencies(first_node_cpp rclcpp) # add rclcpp as a dependency
