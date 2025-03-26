import rclpy # Import the ROS client library
from rclpy.node import Node # Import Node

def main():
    rclpy.init()        # Initialize the ROS client library
    node = Node('first_node_py')    # Create a node 
    node.get_logger().info('first_node_py has been created!')     # Log a message
    rclpy.spin(node)    # Keep the node running
    rclpy.shutdown()    # Shutdown the ROS client library
    
if __name__ == '__main__':
    main()