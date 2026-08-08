import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class CustomNode(Node):
    def __init__(self):
        super().__init__("python_publisher_node")
        
        self.publisher_ = self.create_publisher(
            Int32,
            "first_topic",
            10
        )
        
        self.counter_ = 0
        
        self.timer_ = self.create_timer(
            1,
            self.publisher_callback
        )
    
    def publisher_callback(self):
        message = Int32()
        message.data = self.counter_
        self.counter_ += 1
        self.publisher_.publish(message)
        self.get_logger().info(f"publishing message {message.data}")

def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
