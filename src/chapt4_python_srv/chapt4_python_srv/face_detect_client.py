import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector # 导入自定义服务接口
import cv2 # 导入OpenCV库
import face_recognition # 导入face_recognition库
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # 导入cv_bridge库
import os # 导入os库
import time # 导入time库
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class FaceDetectClient(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.client_ = self.create_client(FaceDetector, 'face_detect') # 创建客户端
        self.bridge = CvBridge() # 创建cv_bridge对象
        self.number_of_times_to_upsample = 1 # 人脸数量
        self.model = 'hog' # 人脸检测模型
        self.image_path = os.path.join(get_package_share_directory('chapt4_python_srv'), 'resource', 'pic1.jpg') # 图片路径
        self.image = cv2.imread(self.image_path) # 读取默认图片

    def call_set_parameters(self, parameters): 
        # 调用set_parameters服务，更新参数，返回服务响应
        update_param_client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        while not update_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetParameters.Request()
        request.parameters = parameters
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    
    def update_detect_model(self, model='hog'):
        # 调用call_set_parameters，更新模型参数
        param=Parameter()
        param.name='model'
        param_value=ParameterValue()
        param_value.string_value=model
        param_value.type=ParameterType.PARAMETER_STRING
        param.value=param_value
            
        response = self.call_set_parameters([param])
        
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'Successfully updated model to {model}')
            else:
                self.get_logger().info('Failed to update model')
            

    def send_request(self):
        while not self.client_.wait_for_service(timeout_sec=1.0):# 等待服务可用
            self.get_logger().info('service not available, waiting again...')
        request = FaceDetector.Request() # 创建请求对象
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client_.call_async(request) # 发送请求并返回未来对象
        rclpy.spin_until_future_complete(self, future) # 等待服务响应
        # future.add_done_callback(self.show_response) # 添加回调函数
        
    def show_response(self, future):
        response=future.result()
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 255, 0), 2) # 在图片上绘制矩形框
        cv2.imshow('image', self.image) # 显示图片
        cv2.waitKey(0) # 等待按键
            
        
def main():
    rclpy.init()
    node = FaceDetectClient('face_detect_client')
    node.update_detect_model('hog')
    node.send_request()
    node.update_detect_model('cnn')
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()