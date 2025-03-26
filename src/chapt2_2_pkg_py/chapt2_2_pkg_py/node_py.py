import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS client library
    node = Node('node_py') # Create a node
    node.get_logger().info('node_py has been created!') # Log a message
    rclpy.spin(node)    # Keep the node running
    node.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()   # Shutdown the ROS client library