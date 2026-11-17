from rclpy.impl.rcutils_logger import RcutilsLogger

import numpy as np
from .robot_confgis_generator import ParseURDF
from .matrix_computation import matrixComputer


class IKSolver():
    def __init__(self, logger: RcutilsLogger, maxIteration: int):
        self.logger_ = logger
        self.maxIteration_: int = maxIteration
        self.theta: np.array = np.array([])
        
        self.urdfParser = ParseURDF(
            "/home/shayo/coding/learn_ros2/src/robot_description/urdf/arm.urdf",
            ["base_to_link_1", "link_1_link_2", "link_2_link_3", "end_effector"]
            )
        self.Slist, self.Blist, self.homeMatrix, self.omegaList, self.vList = self.urdfParser.parse()
        self.matrix = matrixComputer()
        
        self.joint_states_ = np.array([])
        
        # transform matrix given from ros
        self.transformMatrix_ = np.ndarray([])
    
    def set_joint_angles(self, joint_angles: np.array):
        """
        set joint angles
        Args:
            joint_angles (np.array): a array representing joint angles given from ros topic
        """
        self.joint_states_ = joint_angles

    def set_transform_matrix(self, ros2_msg) -> None:
        r = ros2_msg.transform.rotation
        R = self.matrix.quaternion_to_rotation_matrix(r.x, r.y, r.z, r.w)
        t = ros2_msg.transform.translation
        
        self.transformMatrix_ = np.eye(4)
        self.transformMatrix_[:3, :3] = R
        self.transformMatrix_[:3, 3] = [t.x, t.y, t.z]
    
    def computeJointExponential(self, omega: np.ndarray, v: np.ndarray, theta_radian: float) -> np.ndarray:
        """
        [
            [e^[w].theta, t],
            [0      ,     1]
        ]
        Args:
            omega (np.ndarray): angular axes
            v (np.ndarray): linear
            theta_radian (float): joint angle

        Returns:
            T: exponential matrix
        """
        omega_symmetric = self.matrix.compute_symmetric(omega)
        
        rotational = self.matrix.compute_exponential(omega_symmetric, theta_radian)
        
        G = self.matrix.compute_generator_matrix(omega_symmetric, theta_radian)
        
        translational = np.dot(G, v)
        
        T = np.eye(4)
        T[:3, :3] = rotational
        T[:3, 3] = translational
        return T
    
    def computeJointsTransformMatrix(self):
        T_from_body_to_base = np.eye(4)
        for w, v, theta in zip(self.omegaList, self.vList, self.joint_states_):
            T = self.computeJointExponential(w, v, theta)
            T_from_body_to_base = np.dot(T_from_body_to_base, T)
        T_from_body_to_base = np.dot(T_from_body_to_base, self.homeMatrix)
    
    def getTranslateMatrix(self, G: np.ndarray, V: np.array) -> np.array:
        return np.dot(G, V)
    
    def computeErrorMatrix(self, T_from_d_to_s: np.ndarray, T_from_b_to_s: np.ndarray) -> np.ndarray:
        """
        Calculates the relative transformation matrix T_bd from the current pose T_sb 
        to the desired target pose T_sd.
        Formula: T_bd = T_sb^-1 * T_ds
        """
        T_sb_inv = np.linalg.inv(T_from_b_to_s)
        T_from_d_to_b = np.dot(T_sb_inv, T_from_d_to_s)
        return T_from_d_to_b
    
    def getRotationAngle(self, R: np.ndarray) -> float:
        """
        Computes the rotation angle theta in [0, pi] of a 3x3 rotation matrix R.

        Uses theta = atan2(||sin_vec||, cos_part), which stays accurate for
        theta near 0 and theta near pi, where arccos(trace) loses precision.

        Args:
            R (np.ndarray): 3x3 rotation matrix (R^T R = I, det R = 1)

        Returns:
            float: rotation angle theta in radians
        """
        R = np.asarray(R, dtype=float)
        if R.shape != (3, 3):
            raise ValueError(f"R must be 3x3, got shape {R.shape}")

        # cos(theta) = (tr(R) - 1) / 2
        cos_part = (np.trace(R) - 1.0) / 2.0

        # sin(theta) * k = vee(R - R^T) / 2
        sin_vec = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ]) / 2.0
        sin_norm = np.linalg.norm(sin_vec)

        return float(np.arctan2(sin_norm, cos_part))

    def computeOmegaAndOmegaHat(self, R: np.ndarray, theta: float) -> np.ndarray:
        """
        Computes the 3x3 skew-symmetric matrix [w] = theta * [k] from a rotation
        matrix R and its angle theta, where k is the unit rotation axis.
        """
        R = np.asarray(R, dtype=float)

        if np.isclose(theta, 0.0):
            return np.zeros(3), np.zeros((3, 3))

        if np.isclose(theta, 0.0):
            return np.zeros(3), np.zeros((3, 3))
        elif np.isclose(theta, np.pi):
            B = (R + np.eye(3)) / 2.0
            col = B[:, np.argmax(np.diag(B))]
            omega = col / np.linalg.norm(col)
            return omega, self.matrix.compute_symmetric(omega)

        omega_hat = (R - R.T) / (2* np.sin(theta))

        return self.matrix.compute_axes(omega_hat), omega_hat

    def computeTwistError(self, error_matrix: np.ndarray) -> np.ndarray:
        R = error_matrix[0:3, 0:3]
        p = error_matrix[0:3, 3]

        theta, axis = self.getThetaAndOmega(R)
        theta = self.getRotationAngle(R)
        if theta < 1e-8:
            return np.concatenate([np.zeros(3), p])

        w_hat = self.matrix.compute_symmetric(axis)
        theta_Ginv = self.matrix.compute_inverse_generator_matrix(w_hat, theta) * theta
        v = theta_Ginv @ p
        omega = theta * axis
        return np.concatenate([omega, v])  
    
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
    
    def solve_IK(self, TD_S: np.array, theta0: np.array):
        for i in range(self.maxIteration_):
            # transform matrix is given from ros every step
            error_matrix = self.computeErrorMatrix(TD_S, self.transformMatrix_)
            twist_error = self.computeTwistError(error_matrix)
            
            jacobian = self.computeJacobian()
            J_inverse = self.compute_J_inverse(jacobian)
            
            d_theta = twist_error # theta_dot = J_inverse @ Twist error
            self.theta += np.dot(J_inverse, d_theta)
            self.logger_.info(f'{i}')