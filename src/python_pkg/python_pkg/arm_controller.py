import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState


from .modules.IKSolver import IKSolver
import numpy as np
import modern_robotics as mr

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
        
        # joint state publisher
        self.joint_states_publisher_ = self.create_publisher(
            JointState,
            "joint_states",
            10
        )
        
        self.ik_solver = IKSolver(self.get_logger(), 100)
        
        self.is_angles_publised_ = False
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
        self.is_angles_publised_ = True
    
    def run(self):
        # self.tf_listener_()
        if self.is_angles_publised_:
            goal = mr.FKinSpace(self.ik_solver.homeMatrix, self.ik_solver.Slist, np.array([0.25, 0.0, 0.3]))
            self.get_logger().info(f'goal : \n{goal}')
            a = self.ik_solver.solve_IK(goal, pose_tol=10e-5, angle_tol=10e-3)
            self.get_logger().info(f'angles : \n{a}')
            self.is_angles_publised_ = False
        pass



def main(args = None):
    rclpy.init(args = args)
    
    node = CustomNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
