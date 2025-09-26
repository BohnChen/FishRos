import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("python_node")
    node.get_logger().info('hi, This is a Python node!')
    rclpy.spin(node)
    rclpy.shutdown()

