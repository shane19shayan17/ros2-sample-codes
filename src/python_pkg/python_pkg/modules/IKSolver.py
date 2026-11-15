from rclpy.impl.rcutils_logger import RcutilsLogger

import numpy as np


class IKSolver():
    def __init__(self, logger: RcutilsLogger, maxIteration: int):
        self.logger_ = logger
        self.maxIteration_: int = maxIteration
        self.theta: np.array = np.array([])
    
    def computeGoalFromBodyframe(self, T_ds: np.array, T_sb: np.array) -> np.array:
        """
        Calculates the relative transformation matrix T_bd from the current pose T_sb 
        to the desired target pose T_ds.
        Formula: T_bd = T_sb^-1 * T_ds
        """
        T_sb_inv = np.linalg.inv(T_sb)        
        T_bd = np.dot(T_sb_inv, T_ds)
        
        return T_bd
    
    def solve_IK(self, TD_S: np.array, theta0: np.array):
        for i in range(self.maxIteration_):
            d_theta = np.array([]) # should be calculated
            self.theta += d_theta
            self.logger_.info(f'{i}')