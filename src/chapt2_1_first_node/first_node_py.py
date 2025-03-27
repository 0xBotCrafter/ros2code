import rclpy # 导入客户端库
from rclpy.node import Node # 导入Node类

def main():
    rclpy.init()        # 初始化ROS客户端库
    node = Node('first_node_py')    # 创建一个Node对象
    node.get_logger().info('first_node_py has been created!')     # 输出日志信息
    rclpy.spin(node)    # 进入循环，阻塞直到节点被关闭
    rclpy.shutdown()    # 关闭ROS客户端库
    
if __name__ == '__main__':
    main()