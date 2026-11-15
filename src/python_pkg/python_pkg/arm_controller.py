import rclpy
from rclpy.node import Node

from .modules.IKSolver import IKSolver
import numpy as np


class CustomNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.ik_solver = IKSolver(self.get_logger(), 100)
        
        a = np.array([
            [0,1],
            [1,2],
            ])
        b = np.array([
                    [1,2],
                    [1,1],
                    ])
        print(self.ik_solver.computeGoalFromBodyframe(a, b))
        
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
