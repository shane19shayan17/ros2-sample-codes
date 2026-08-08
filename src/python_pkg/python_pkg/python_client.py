import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

import sys


class PythonClient(Node):
    def __init__(self):
        super().__init__("python_client_node")
        
        self.client_ = self.create_client(
            SetBool,
            "first_service",
        )
        
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting ...')
    
    
    def request_callback(self, data : bool):
        request = SetBool.Request()
        request.data = data
        self.future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args = None):
    rclpy.init(args = args)
    
    node = PythonClient()
    response = node.request_callback(
        bool(sys.argv[1])
    )
    node.get_logger().info(f"success -> {response.success} || message -> {response.message}")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
