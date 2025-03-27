#include <iostream> // 包含iostream头文件
#include "rclcpp/rclcpp.hpp" // 包含ROS2的头文件

int main(int argc, char **argv) { // 主函数
    rclcpp::init(argc, argv); // 初始化ROS2
    auto node = std::make_shared<rclcpp::Node>("first_node_cpp"); // 创建一个节点
    RCLCPP_INFO(node->get_logger(), "first_node_cpp has been created!");// 输出信息
    rclcpp::spin(node); // 阻塞，直到节点被关闭
    rclcpp::shutdown(); // 关闭ROS2
    
    return 0;
}