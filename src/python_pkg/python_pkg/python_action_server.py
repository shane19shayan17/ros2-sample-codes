import rclpy
from rclpy.node import Node

from messages.action import Count

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import time

class ActionNode(Node):
    def __init__(self):
        super().__init__("python_action_server")
        self.counter_action_server_ = ActionServer(
            self,
            Count,
            "counter",
            execute_callback=self.execute_couner_action_callback
        )
    
    def execute_couner_action_callback(self, goalHandle : ServerGoalHandle):
        # get request from goal
        goal = goalHandle.request.goal
        
        # execute the goal
        self.get_logger().info(f'action executing with goal : {goal}')
        counter = 1
        for i in range(goal):
            counter * i
            time.sleep(1)
        # set goal final stste
        goalHandle.succeed()
        
        # send result
        
        result = Count.Result()
        result.reached = counter
        return result

def main(args = None):
    rclpy.init(args = args)
    
    node = ActionNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
