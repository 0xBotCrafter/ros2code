#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) { // argc: argument count, argv: argument vector
    rclcpp::init(argc, argv); // initialize ROS2
    auto node = std::make_shared<rclcpp::Node>("first_node_cpp"); // create a node
    RCLCPP_INFO(node->get_logger(), "first_node_cpp has been created!");           // print "Hello, world!" to the console
    rclcpp::spin(node); // spin the node
    rclcpp::shutdown(); // shutdown ROS2
    
    return 0;
}