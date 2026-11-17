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
                
        self.create_timer(
            0.5,
            self.run
        )
    
    def tf_listener_(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', # name of base link in urdf
                'end_effector', # name of end effector in urdf
                rclpy.time.Time()
            )
            self.ik_solver.set_transform_matrix(t)
        except Exception as ex:
            self.get_logger().info(f'Could not transform: {ex}')
    
    def joint_states_callback(self, msg: JointState):
        self.ik_solver.set_joint_angles(np.asarray(msg.position))
        self.ik_solver.computeJointsTransformMatrix()
    
    def run(self):
        self.tf_listener_()
        pass



def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
