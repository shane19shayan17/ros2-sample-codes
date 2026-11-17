import numpy as np

class matrixComputer():
    def __init__(self):
        pass
    
    def compute_symmetric(self, matrix: np.array):
        x, y, z = matrix
        return np.array([
            [ 0.0, -z,  y],
            [ z,  0.0, -x],
            [-y,  x,  0.0]
        ])
    
    def compute_axes(self, matrix: np.ndarray):
        return np.array(
            [matrix[2, 1] ,matrix[0, 2], matrix[1, 0]]
        )
    
    def compute_exponential(self, symmetric_matrix: np.ndarray, theta_radian: float):
        size_ = symmetric_matrix.shape[0]
        w_hat_2 = np.eye(size_) \
            + np.sin(theta_radian) * symmetric_matrix \
                + (1 - np.cos(theta_radian)) * np.dot(symmetric_matrix, symmetric_matrix)
        return w_hat_2
    
    def compute_generator_matrix(self, symmetric_matrix: np.ndarray, theta_radian: float):
        size_ = symmetric_matrix.shape[0]
        G = np.eye(size_) * theta_radian \
            + (1 - np.cos(theta_radian)) * symmetric_matrix \
                + (theta_radian - np.sin(theta_radian)) * np.dot(symmetric_matrix, symmetric_matrix)
        return G
    
    def compute_inverse_generator_matrix(self, omega_hat: np.ndarray, theta_radian: float):
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
        if np.isclose(theta_radian, 0.0):
            return np.eye(size_)
            
        # Calculate cotangent of half the angle
        cot_half = 1.0 / np.tan(theta_radian / 2.0)
        
        G_inv = (
            (1.0 / theta_radian) * np.eye(size_) - 0.5 * omega_hat + (1.0 / theta_radian - 0.5 * cot_half) * np.dot(omega_hat, omega_hat)
        )
        return G_inv
    
    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """Convert a quaternion (x, y, z, w) to a 3x3 rotation matrix.

        The quaternion order matches ROS (x, y, z, w).
        """
        # Normalize: numerical noise can make the quaternion slightly non-unit,
        # and a non-unit quaternion produces a matrix that is not a pure rotation.
        norm = np.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            raise ValueError("Quaternion has zero length")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])
        
        
    
if __name__ == "__main__":
    m = matrixComputer()
    v = np.array([1, 2, 3])
    w = m.compute_symmetric(v)
    ew = m.compute_exponential(w, np.pi / 2)
    g = m.compute_generator_matrix(w, np.pi / 2)
    print(g)