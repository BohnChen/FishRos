import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # create Queue to save the novel
        self.novels_queue_ = Queue() 
        # Create topic publisher to publish novel
        self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
        # create timer
        self.timer_ = self.create_timer(5, self.timer_callback)

    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        self.get_logger().info(f'download complete: {url}')
        # split according to line, put into queue
        for line in response.text.splitlines():
            self.novels_queue_.put(line)

    def timer_callback(self) :
        # There are msgs in the queue, get and publish a line
        if self.novels_queue_.qsize() > 0:
            # Instantiate a message
            msg = String()
            # 对消息结构体进行赋值
            msg.data = self.novels_queue_.get()
            # 发布消息
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f'发布了一行小说: {msg.data}')

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    node.download_novel('http://localhost:8000/novel1.txt')
    rclpy.spin(node)
    rclpy.shutdown()

