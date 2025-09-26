import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self,node_name:str, name:str, age:int) -> None:
        super().__init__(node_name)
        #print('PresonNode\'s __init__ function has been used.' )
        self.age = age
        self.name = name

    def eat(self, food_name:str):
        self.get_logger().info(f'My name is {self.name}, now I am {self.age} years old, and I am eating {food_name}')
        print(f'my name is {self.name}, now , I am {self.age} years old, I am eating {food_name}')

def main():
    rclpy.init()
    node = PersonNode('person_node', 'ZhangSan', '18')
    node.eat('Ros')
    rclpy.spin(node)
    rclpy.shutdown()
