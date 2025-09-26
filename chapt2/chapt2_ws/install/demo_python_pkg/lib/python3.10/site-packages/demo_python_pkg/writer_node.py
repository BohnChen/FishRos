
import rclpy
#import demo_python_pkg.person_node
from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode) :
    def __init__(self,node_name:str, name: str, age:int,  book:str) -> None:
        super().__init__(node_name, name, age)
        print('WriterNode\'s __init__ function has been used!')
        self.book = book
        #self.get_logger().info(f'My name is {self.name},now is {self.age} years old.')

def main():
    rclpy.init()
    node = WriterNode('writer_node','ZhangSi',18,'Zhand San selfy')
    node.eat('Ros Ros')
    rclpy.spin(node)
    rclpy.shutdown()
