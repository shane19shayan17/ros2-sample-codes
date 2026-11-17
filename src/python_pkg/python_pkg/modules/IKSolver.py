from rclpy.impl.rcutils_logger import RcutilsLogger

import numpy as np
from .robot_confgis_generator import ParseURDF


class IKSolver():
    def __init__(self, logger: RcutilsLogger, maxIteration: int):
        self.logger_ = logger
        self.maxIteration_: int = maxIteration
        self.theta: np.array = np.array([])
        
        self.urdfParser = ParseURDF(
            "/home/shayo/coding/learn_ros2/src/robot_description/urdf/arm.urdf",
            ["base_to_link_1", "link_1_link_2", "link_2_link_3", "end_effector"]
            )
        self.Slist, self.Blist, self.M = self.urdfParser.parse()
    
    def getSkewSymmetric(self, T: np.array):
        """
        Converts a 3D vector v = [v_x, v_y, v_z] into a 3x3 skew-symmetric matrix [v].
        
        Parameters:
        - v: numpy array or list of shape (3,)
        
        Returns:
        - A 3x3 numpy array representing the skew-symmetric matrix.
        """
        vx, vy, vz = T
        return np.array([
            [ 0.0, -vz,  vy],
            [ vz,  0.0, -vx],
            [-vy,  vx,  0.0]
        ])
    
    def getGMatrix(self, theta: float, twist_symmetric: np.ndarray) -> np.ndarray:
        """
        Computes the generator matrix G(theta) for the translation component.
        Formula: G(theta) = I*theta + (1 - cos(theta))*[omega] + (theta - sin(theta))*[omega]^2
        """
        size_ = twist_symmetric.shape[0] 
        
        return (
            theta * np.eye(size_) 
            + (1.0 - np.cos(theta)) * twist_symmetric 
            + (theta - np.sin(theta)) * np.dot(twist_symmetric, twist_symmetric)
        )
    
    def getTranslateMatrix(self, G: np.ndarray, V: np.array) -> np.array:
        return np.dot(G, V)
    
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
    
    def computeAdjoint(self, T: np.ndarray) -> np.ndarray:
        """
        Computes the 6x6 Adjoint matrix for a 4x4 transformation matrix T.
        Ad_T = [[R, 0], 
                [[p]_hat @ R, R]]
        """
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        
        # Create the 3x3 skew-symmetric matrix [p]
        p_hat = np.array([
            [0, -p[2], p[1]],
            [p[2], 0, -p[0]],
            [-p[1], p[0], 0]
        ])
        
        # Assemble the 6x6 Adjoint matrix
        zero_3x3 = np.zeros(R.shape)
        ad_T = np.block([
            [R, zero_3x3],
            [np.dot(p_hat, R), R]
        ])
        
        return ad_T
    
    def matrix_exp(skew_symmetric_matrix: np.ndarray, theta: float) -> np.ndarray:
        """
        Computes the matrix exponential e^([B] * theta) for a 4x4 twist matrix [B].
        [B] = [[omega_hat, v], [0, 0]]
        """
        
        # Extract angular velocity vector omega from omega_hat
        omega = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
        omega_norm = np.linalg.norm(omega)
        
        T = np.eye(4)
        
        # Pure translation case
        if np.isclose(omega_norm, 0.0):
            T[0:3, 3] = v * theta
        else:
            # Normalized rotation axis and scaled angle
            omega_axis = omega / omega_norm
            theta_scaled = omega_norm * theta
            
            
            # Rodrigues' formula for rotation R
            R = (
                np.eye(3) 
                + np.sin(theta_scaled) * skew_symmetric_matrix
                + (1.0 - np.cos(theta_scaled)) * np.dot(skew_symmetric_matrix, skew_symmetric_matrix)
            )
            return R
            
            # Generator matrix G for translation component p
            G = (
                np.eye(3) * theta_scaled 
                + (1.0 - np.cos(theta_scaled)) * omg_mat 
                + (theta_scaled - np.sin(theta_scaled)) * np.dot(omg_mat, omg_mat)
            )
            p = np.dot(G, v)
            
            T[0:3, 0:3] = R
            T[0:3, 3] = p
            
        return T
    
    def computeTransform(self):
        pass
    
    def computeJacobian(self) -> np.ndarray:
        pass
    
    def compute_J_inverse(self, J: np.ndarray) -> np.ndarray:
        pass
    
    def solve_IK(self, TD_S: np.array, T_SB, theta0: np.array):
        for i in range(self.maxIteration_):
            t_from_d_to_b = self.computeGoalFromBodyframe(TD_S, T_SB)
            twist_error = self.computeErrorTwist(t_from_d_to_b)
            
            jacobian = self.computeJacobian()
            J_inverse = self.compute_J_inverse(jacobian)
            
            d_theta = twist_error # theta_dot = J_inverse @ Twist error
            self.theta += np.dot(J_inverse, d_theta)
            self.logger_.info(f'{i}')