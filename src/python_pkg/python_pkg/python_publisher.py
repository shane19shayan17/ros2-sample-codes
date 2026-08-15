import rclpy
from rclpy.node import Node

from messages.msg import Velocity

class CustomNode(Node):
    def __init__(self):
        super().__init__("python_publisher_node")
        
        self.declare_parameter("frequency", 1.0)
        self.frequency_ = self.get_parameter("frequency").value
        
        self.publisher_ = self.create_publisher(
            Velocity,
            "first_topic",
            10
        )
        
        self.counter_ = 0.0
        
        self.timer_ = self.create_timer(
            self.frequency_,
            self.publisher_callback
        )
    
    def publisher_callback(self):
        message = Velocity()
        message.velocity_x = self.counter_ * 100
        message.velocity_y = self.counter_ * 10
        message.velocity_z = self.counter_
        self.counter_ += 1
        self.publisher_.publish(message)
        self.get_logger().info(f"publishing message x:{message.velocity_x}, y:{message.velocity_y}, z:{message.velocity_z}")

def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
