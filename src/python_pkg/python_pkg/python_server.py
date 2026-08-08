import rclpy
from rclpy.node import Node

from example_interfaces.srv import SetBool

class PythonServer(Node):
    def __init__(self, node_name : str):
        super().__init__(node_name = node_name)
        
        self.service_ = self.create_service(
            SetBool, "set_bool", self.set_bool_callback)
        
    def set_bool_callback(self, request, response):
        response.success = True
        response.message = f"bool has seted to {request.data} successfully"
        return response


def main(args = None):
    rclpy.init(args = args)
    
    node = PythonServer("python_server_node")
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
