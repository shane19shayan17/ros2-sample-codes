import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState


from .modules.IKSolver import IKSolver
import numpy as np


class CustomNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # tf from ros
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_angles_sub_ = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            10
        )
        
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
        e = self.ik_solver.getTranslateMatrix(d, np.array([2, 0, 3]))
        # self.get_logger().info(f'\n{e}')
        a = self.ik_solver.computeJointExponential(self.ik_solver.omegaList[1], self.ik_solver.vList[1], 90)
        self.get_logger().info(f'\n{a}')
        
        
        self.create_timer(
            0.5,
            self.run
        )
    
    def tf_listener_(self):
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         'base_link', # name of base link in urdf
        #         'end_effector', # name of end effector in urdf
        #         rclpy.time.Time()
        #     )
        #     end_effector_pose: np.ndarray = np.ndarray([])
        #     # t.transform.rotation
        #     self.get_logger().info(
        #         f'X: {t.transform.translation.x:.3f}, '
        #         f'Y: {t.transform.translation.y:.3f}, '
        #         f'Z: {t.transform.translation.z:.3f}, '
        #         f'x: {t.transform.rotation.x:.3f}, '
        #         f'y: {t.transform.rotation.y:.3f}, '
        #         f'z: {t.transform.rotation.z:.3f}'
        #     )
        # except Exception as ex:
        #     self.get_logger().info(f'Could not transform: {ex}')
        pass
    
    def joint_states_callback(self, msg: JointState):
        self.ik_solver.set_joint_angles(np.asarray(msg.position))
    
    def run(self):
        self.tf_listener_()



def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
