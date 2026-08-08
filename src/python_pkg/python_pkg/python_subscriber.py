import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class SubscriberNode(Node):
    def __init__(self):
        super().__init__("python_subscriber_node")
        
        self.subscriber_ = self.create_subscription(
            Int32,
            "first_topic",
            self.subscriber,
            10
        )
        
        self.msg = None


    def subscriber(self, msg : Int32):
        self.msg = msg.data
        self.get_logger().info(f"msg received -> {self.msg}")

def main(args = None):
    rclpy.init(args = args)
    
    node = SubscriberNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
