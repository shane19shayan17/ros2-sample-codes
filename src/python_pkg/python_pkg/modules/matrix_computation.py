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
    
    def compute_joint_exponential(self, rotational: np.ndarray, tranlatinal: np.ndarray):
        T = np.eye(4)
        T[:3, :3] = rotational
        T[:3, 3] = tranlatinal
        return T
    
if __name__ == "__main__":
    m = matrixComputer()
    v = np.array([1, 2, 3])
    w = m.compute_symmetric(v)
    ew = m.compute_exponential(w, np.pi / 2)
    g = m.compute_generator_matrix(w, np.pi / 2)
    print(g)