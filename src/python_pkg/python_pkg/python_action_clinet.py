import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from messages.action import Count

class ActionClientNode(Node):
    def __init__(self):
        super().__init__("python_action_clinet_node")
        
        self.action_clinet_ = ActionClient(
            self,
            Count,
            "counter"
        )
    
    def send_goal(self, goal):
        # waiting for server
        self.action_clinet_.wait_for_server()
        
        # create goal
        goal_msg = Count.Goal()
        goal_msg.goal = goal
        
        # return goal
        return self.action_clinet_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.done_callback)
    
    def feedback_callback(self, feedback : Count.Feedback):
        self.get_logger().info(f'feedback -> {feedback.feedback.current}')
    
    def done_callback(self, future):
        goal_handle : ClientGoalHandle = future.result()
        if goal_handle.accepted:
            goal_handle.get_result_async().add_done_callback(self.goal_done_callback)
            self.get_logger().info(f'action accepted')
        else:
            self.get_logger().info(f'action rejected')

    def goal_done_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'the action finished but aborted !')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'the action finished succeed!')
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'goal cenceled')

def main(args = None):
    rclpy.init(args = args)
    
    node = ActionClientNode()
    node.send_goal(6)
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
