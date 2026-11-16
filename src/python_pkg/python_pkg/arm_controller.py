import rclpy
from rclpy.node import Node

from .modules.IKSolver import IKSolver
import numpy as np


class CustomNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.ik_solver = IKSolver(self.get_logger(), 100)
        
        # a = self.ik_solver.computeAdjoint(np.array([
        #     [0, -1, 0, 2],
        #     [1, 0, 0, 0],
        #     [0, 0, 1, 3],
        #     [0, 0, 0,1],
        #     ]))
        b = np.array([1, 2, 3])
        c = self.ik_solver.getSkewSymmetric(b)
        d = self.ik_solver.getGMatrix(np.pi / 2, c)
        self.get_logger().info(f'\n{d}')
        
        self.create_timer(
            0.5,
            self.run
        )
    
    def run(self):
        pass

def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
