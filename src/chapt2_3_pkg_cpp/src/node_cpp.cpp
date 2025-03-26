#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_cpp");
    RCLCPP_INFO(node->get_logger(), "node_cpp has been created!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}