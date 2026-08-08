import rclpy
from rclpy.node import Node

from example_interfaces.msg import String, Int32 ,_int32

class SubscriberNode(Node):
    def __init__(self, node_name : str):
        super().__init__(node_name = node_name)

        self.subscriber_ = self.create_subscription(Int32, "second_topic", self.subscriber, 5)
        self.msg = None


    def subscriber(self, msg : Int32):
        self.msg = msg.data
        self.get_logger().info(f"msg -> {self.msg}")

def main(args = None):
    rclpy.init(args = args)
    
    node = SubscriberNode("subscriber_node")
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
