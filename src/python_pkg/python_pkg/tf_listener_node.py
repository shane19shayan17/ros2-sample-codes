import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class EndEffectorTFListener(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Query the transform every second
        self.timer = self.create_timer(0.50, self.tf_listener_callback)

    def tf_listener_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', # name of base link in urdf
                'end_effector', # name of end effector in urdf
                rclpy.time.Time()
            )
            self.get_logger().info(
                f'Translation - X: {t.transform.translation.x:.3f}, '
                f'Y: {t.transform.translation.y:.3f}, '
                f'Z: {t.transform.translation.z:.3f}'
            )
        except Exception as ex:
            self.get_logger().info(f'Could not transform: {ex}')

def main():
    rclpy.init()
    node = EndEffectorTFListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()