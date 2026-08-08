import rclpy
from rclpy.node import Node


class CustomNode(Node):
    def __init__(self, node_name : str):
        super().__init__(node_name = node_name)

def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode("template")
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
