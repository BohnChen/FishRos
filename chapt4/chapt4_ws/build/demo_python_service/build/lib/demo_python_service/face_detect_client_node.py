import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge


class FaceDetectorClient(Node) :
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        # self.test1_image_path = get_package_share_directory( 'demo_python_service') + '/resource/test1.png'
        self.test1_image_path = get_package_share_directory( 'demo_python_service') + '/resource/test1.png'
        self.image = cv2.imread(self.test1_image_path)

    def send_request(self) :
        # TODO: 发送请求并处理结果
        # 1. 判断服务是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f"等待服务端上线...")
        # 2. 构造 Request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # 3. 发送并spin 等待服务处理完成
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # 4. 根据处理结果
        response = future.result()
        self.get_logger().info(
            f'接收到响应： 图像中共有： {response.number} 张脸， 耗时{response.use_time}')
        self.show_face_locations(response)

    #def send_request(self) :
    #    # 1. 判断服务是否上线
    #    while self.client.wait_for_service(timeout_sec=1.0) is False:
    #        self.get_logger().info(f"等待服务端上线...")
    #    # 2. 构造 Request
    #    request = FaceDetector.Request()
    #    request.image = self.bridge.cv2_to_imgmsg(self.image)
    #    # 3. 发送并spin 等待服务处理完成
    #    future = self.client.call_async(request)
    #    def request_callback(result_future):
    #        response = result_future.result()
    #        self.get_logger().info(f'接收到响应： 图像中共有： {response.number} 张脸， 耗时{response.use_time}')
    #        self.show_face_locations(response)
    #    future.add_done_callback(request_callback)


    def show_face_locations(self, response) :
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom [i]
            left = response.left [i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 2)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)

def main(args=None) :
    rclpy.init(args=args)
    face_detect_client = FaceDetectorClient()
    face_detect_client.send_request()
    #rclpy.spin(face_detect_client)
    rclpy.shutdown()
