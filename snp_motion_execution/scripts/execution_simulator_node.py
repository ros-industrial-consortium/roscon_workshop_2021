#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.action import FollowJointTrajectory


class ExecSimServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.FJT_ACTION_PARAM = "follow_joint_trajectory_action"
        fjt_action =  self.declare_parameter(self.FJT_ACTION_PARAM, 
                                             "joint_trajectory_action").value
        self.action_server = ActionServer(self, 
                                          FollowJointTrajectory, 
                                          fjt_action,
                                          execute_callback=self.execute_cb,
                                          goal_callback=self.goal_cb,
                                          cancel_callback=self.cancel_cb)

        self.get_logger().info("Started simulated robot execution node")

    '''Callback to execute action'''
    async def execute_cb(self, goal_handle):
        self.get_logger().info("Executing goal")
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed() 
        self.get_logger().info("Goal succeeded")
        return result

    '''Callback to accept action.'''
    def goal_cb(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT
    
    '''Callback to cancel action'''
    def cancel_cb(self, goal_handle):
        self.get_logger().info("Received request to cancel goal")
        return CancelResponse.ACCEPT

if __name__ == "__main__":
    rclpy.init()

    node = ExecSimServer("motion_execution_server_sim")

    rclpy.spin(node)

    rclpy.shutdown()