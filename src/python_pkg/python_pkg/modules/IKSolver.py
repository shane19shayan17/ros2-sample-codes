from rclpy.impl.rcutils_logger import RcutilsLogger

import numpy as np


class IKSolver():
    def __init__(self, logger: RcutilsLogger, maxIteration: int):
        self.logger_ = logger
        self.maxIteration_: int = maxIteration
        self.theta: np.array = np.array([])
    
    def computeGoalFromBodyframe(self, T_from_d_to_s: np.ndarray, T_from_b_to_s: np.ndarray) -> np.ndarray:
        """
        Calculates the relative transformation matrix T_bd from the current pose T_sb 
        to the desired target pose T_sd.
        Formula: T_bd = T_sb^-1 * T_ds
        """
        T_sb_inv = np.linalg.inv(T_from_b_to_s)
        T_from_d_to_b = np.dot(T_sb_inv, T_from_d_to_s)
        return T_from_d_to_b
    
    def getTheta(self, R: np.ndarray) -> float:
        cos_theta = (np.trace(R) - 1) / 2.0
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        return np.arccos(cos_theta)

    def getOmegaAndOmegaHat(self,theta: float, R: np.ndarray) -> tuple:
        """
        calculating skew-symmetric from rotational matrix
        Args:
            R (np.ndarray): rotational matrix

        Returns:
            np.array: omega matrix which is the skew-symmetric
        """        
        # handle pure rotations
        if np.isclose(theta, 0.0):
            return np.zeros(R.shape[0]), np.zeros(R.shape)
        
        omega_hat = (R - R.T) / (2 * np.sin(theta))
        omega = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
        return omega, omega_hat
    
    def getGInverseMatrix(self, theta: float, omega_hat: np.ndarray) -> np.ndarray:
        """
        Computes the inverse generator matrix G^-1(theta) for SE(3) matrix logarithm.
        
        Parameters:
        - theta: float, the rotation angle in radians.
        - omega_hat: np.ndarray, the 3x3 skew-symmetric matrix of the rotation axis [omega].
        
        Returns:
        - G_inv: nxn (3X3) numpy array representing G^{-1}(theta).
        """
        size_ = omega_hat.shape[0]
        # Handle the numerical singularity or zero rotation case
        if np.isclose(theta, 0.0):
            return np.eye(size_)
            
        # Calculate cotangent of half the angle
        cot_half = 1.0 / np.tan(theta / 2.0)
        
        G_inv = (
            (1.0 / theta) * np.eye(size_) 
            - 0.5 * omega_hat 
            + (1.0 / theta - 0.5 * cot_half) * np.dot(omega_hat, omega_hat)
        )
        return G_inv

    def computeErrorTwist(self, T_bd: np.ndarray) -> np.ndarray:
        """
        Computes the matrix logarithm (error twist V_b) from T_bd using pure NumPy.
        Dennotes T_bd as [[R, p], [0, 1]].
        """
        R = T_bd[0:3, 0:3]
        p = T_bd[0:3, 3]

        theta = self.getTheta(R)
        
        omega, omega_hat = self.getOmegaAndOmegaHat(theta, R)
        G_inverse = self.getGInverseMatrix(theta, omega_hat)
        v = np.dot(G_inverse, p)
        V_b = np.concatenate([omega, v])
        
        return V_b
    
    def solve_IK(self, TD_S: np.array, theta0: np.array):
        for i in range(self.maxIteration_):
            d_theta = np.array([]) # should be calculated
            self.theta += d_theta
            self.logger_.info(f'{i}')