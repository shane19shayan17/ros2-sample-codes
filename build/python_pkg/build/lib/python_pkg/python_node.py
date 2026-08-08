import rclpy
from rclpy.node import Node

from example_interfaces.msg import String, Int32

class PyNode(Node):
    def __init__(self, node_name : str):
        super().__init__(node_name = node_name)
        
        self.count_ = 0
        
        self.publisher_ = self.create_publisher(Int32, "python_publisher", 5)
        self.create_timer(1, self.publisher)
    
    def publisher(self):
        massage = Int32()
        massage.data = self.count_
        self.count_ += 1
        self.publisher_.publish(massage)
        self.get_logger().info("i am publishing")

def main(args = None):
    rclpy.init(args = args)
    
    node = PyNode("python_node")
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
