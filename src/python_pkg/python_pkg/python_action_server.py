import rclpy
from rclpy.node import Node

from messages.action import Count

from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle

import time

class ActionNode(Node):
    def __init__(self):
        super().__init__("python_action_server")
        self.counter_action_server_ = ActionServer(
            self,
            Count,
            "counter",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_couner_action_callback
        )
        self.goal_handle_: ServerGoalHandle = None
    
    def goal_callback(self, goal_request : Count.Goal):
        self.get_logger().info(f'goal recived !')
        
        if self.goal_handle_ is not None:
            if self.goal_handle_.is_active:
                self.get_logger().warn(f'i got new goal while executing previous goal ...!')
                return GoalResponse.REJECT
        if goal_request.goal <= 0:
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT
    
    def execute_couner_action_callback(self, goalHandle : ServerGoalHandle):
        self.goal_handle_ = goalHandle
        
        # get request from goal
        goal = goalHandle.request.goal
        
        # execute the goal
        feedback = Count.Feedback()
        counter = 1
        for i in range(1, goal + 1):
            counter *= i
            feedback.current = counter
            goalHandle.publish_feedback(feedback)
            self.get_logger().info(f'counter : {counter}')
            time.sleep(1.5)
        # set goal final stste
        if counter < 0:
            goalHandle.abort()
        else:
            goalHandle.succeed()
        
        # send result
        result = Count.Result()
        result.message = "we are so cool so we can learn ros action !"
        result.reached = counter
        return result

def main(args = None):
    rclpy.init(args = args)
    
    node = ActionNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
