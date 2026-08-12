import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty


class EmptyClient(Node):
    def __init__(self):
        super().__init__("python_client_empty_node")
        self.client_ = self.create_client(
            Empty,
            "first_service"
        )
        
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info('service not available yet, waiting ...')
    
    
    def send_request(self):
        request = Empty.Request()
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args = None):
    rclpy.init(args = args)
    
    node = EmptyClient()
    node.send_request()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
