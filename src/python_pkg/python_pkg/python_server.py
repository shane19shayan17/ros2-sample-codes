import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class PythonServer(Node):
    def __init__(self):
        super().__init__("python_server_node")
        
        self.service_ = self.create_service(
            SetBool,
            "first_service",
            self.set_bool_callback)
        
    def set_bool_callback(self, request, response):
        response.success = True
        response.message = f"bool has seted to {request.data} successfully"
        
        self.get_logger().info(f"msg -> {response.message}, success -> {response.success}")
        return response


def main(args = None):
    rclpy.init(args = args)
    
    node = PythonServer()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
