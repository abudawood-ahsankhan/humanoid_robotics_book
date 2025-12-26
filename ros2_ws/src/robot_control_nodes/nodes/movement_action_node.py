#!/usr/bin/env python3

"""
Movement Action Server Node for ROS 2 Robotics Module
This node provides an action server for moving the robot to a target pose
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from robot_control_nodes.action import MoveRobot
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
import math


class MovementActionNode(Node):
    def __init__(self):
        super().__init__('movement_action_node')

        # Create action server for robot movement
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publisher for joint commands to control the robot
        self.joint_cmd_publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )

        self.get_logger().info('Movement Action Node has been started')

    def goal_callback(self, goal_request):
        """Accept or reject a goal request"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the movement action"""
        self.get_logger().info('Executing movement action...')

        # Get the target pose from the goal
        target_pose = goal_handle.request.target_pose

        # Publish feedback periodically
        feedback_msg = MoveRobot.Feedback()
        feedback_msg.distance_to_goal = 1.0  # Initial distance
        feedback_msg.current_status = 'Moving to target pose'

        # For this simple implementation, we'll just move the legs to simulate movement
        # In a real robot, this would involve complex locomotion planning
        joint_cmd = JointState()
        joint_cmd.name = ['left_hip_pitch', 'right_hip_pitch', 'left_knee', 'right_knee']

        # Simulate walking motion
        for step in range(10):  # 10 steps to "reach" the target
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = MoveRobot.Result()
                result.success = False
                result.message = 'Goal canceled'
                return result

            # Alternate leg movement to simulate walking
            if step % 2 == 0:
                joint_cmd.position = [0.2, -0.2, 0.3, 0.0]
            else:
                joint_cmd.position = [-0.2, 0.2, 0.0, 0.3]

            self.joint_cmd_publisher.publish(joint_cmd)

            # Update feedback
            feedback_msg.distance_to_goal = 1.0 - (step * 0.1)
            feedback_msg.current_status = f'Moving... Step {step+1}/10'
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.5)  # Wait before next step

        # Once we've "reached" the target, hold the position
        joint_cmd.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)

        # Check if goal was canceled during execution
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            result = MoveRobot.Result()
            result.success = False
            result.message = 'Goal canceled'
            return result

        # Complete the goal
        goal_handle.succeed()
        result = MoveRobot.Result()
        result.success = True
        result.message = f'Reached target pose: ({target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z})'

        self.get_logger().info(f'Movement action completed: {result.message}')
        return result


def main(args=None):
    rclpy.init(args=args)

    movement_action_node = MovementActionNode()

    try:
        rclpy.spin(movement_action_node)
    except KeyboardInterrupt:
        pass
    finally:
        movement_action_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()