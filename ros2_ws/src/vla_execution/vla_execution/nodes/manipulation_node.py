#!/usr/bin/env python3

"""
Manipulation Node for VLA System

This node handles manipulation actions for grasping and placement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from vla_msgs.msg import GraspRequest, GraspResult, ExecutionStatus
from vla_msgs.action import GraspAction
from rclpy.action import ActionServer, GoalResponse, CancelResponse


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        # Create subscriber for grasp requests
        self.grasp_request_sub = self.create_subscription(
            GraspRequest,
            '/vla/grasp_request',
            self.grasp_request_callback,
            10
        )

        # Create publisher for grasp results
        self.grasp_result_pub = self.create_publisher(GraspResult, '/vla/grasp_result', 10)

        # Create publisher for execution status
        self.status_pub = self.create_publisher(ExecutionStatus, '/vla/execution_status', 10)

        # Create action server for grasp actions
        self.grasp_action_server = ActionServer(
            self,
            GraspAction,
            'grasp_object',
            execute_callback=self.execute_grasp_callback,
            goal_callback=self.grasp_goal_callback,
            cancel_callback=self.grasp_cancel_callback
        )

        # Manipulation state
        self.current_grasp_request = None
        self.manipulation_active = False

        self.get_logger().info('Manipulation Node initialized')

    def grasp_request_callback(self, msg):
        """Handle incoming grasp request"""
        self.get_logger().info(f'Received grasp request for object: {msg.target_object.class_name}')
        self.current_grasp_request = msg

        # Process the grasp request
        result = self.execute_grasp(msg)

        # Publish result
        self.grasp_result_pub.publish(result)

    def execute_grasp_callback(self, goal_handle):
        """Execute grasp action callback"""
        self.get_logger().info('Executing grasp action')
        feedback_msg = GraspAction.Feedback()
        result_msg = GraspAction.Result()

        # Publish initial feedback
        feedback_msg.status = 'approaching_object'
        feedback_msg.progress = 0.2
        feedback_msg.current_step = 'Moving to approach pose'
        goal_handle.publish_feedback(feedback_msg)

        # Simulate grasp execution
        self.get_logger().info('Approaching object...')
        # In real system, this would move the arm to approach position

        # Publish feedback
        feedback_msg.status = 'grasping'
        feedback_msg.progress = 0.6
        feedback_msg.current_step = 'Executing grasp'
        goal_handle.publish_feedback(feedback_msg)

        # In real system, this would execute the grasp
        success = True  # Simulate success

        if success:
            feedback_msg.status = 'lifting'
            feedback_msg.progress = 0.9
            feedback_msg.current_step = 'Lifting object'
            goal_handle.publish_feedback(feedback_msg)

            result_msg.success = True
            result_msg.result_message = 'Grasp successful'
            result_msg.grasp_force_applied = 10.0
            result_msg.confidence = 0.95

            goal_handle.succeed()
        else:
            result_msg.success = False
            result_msg.result_message = 'Grasp failed'
            result_msg.grasp_force_applied = 0.0
            result_msg.confidence = 0.1

            goal_handle.abort()

        # Publish execution status
        status_msg = ExecutionStatus()
        status_msg.plan_id = 'current_plan'  # Would be actual plan ID in real implementation
        status_msg.action_id = 'manipulation_action'
        status_msg.status = 'completed' if success else 'failed'
        status_msg.progress = 1.0 if success else 0.0
        status_msg.start_time = self.get_clock().now().to_msg()
        status_msg.estimated_completion = self.get_clock().now().to_msg()
        status_msg.feedback = [f'Grasp action completed with result: {result_msg.result_message}']

        self.status_pub.publish(status_msg)

        return result_msg

    def grasp_goal_callback(self, goal_request):
        """Handle grasp goal request"""
        self.get_logger().info('Received grasp goal request')
        return GoalResponse.ACCEPT

    def grasp_cancel_callback(self, goal_handle):
        """Handle grasp cancel request"""
        self.get_logger().info('Received grasp cancel request')
        return CancelResponse.ACCEPT

    def execute_grasp(self, request):
        """Execute a grasp request"""
        result = GraspResult()
        result.request_id = request.request_id

        # Simulate grasp execution
        success = True  # In real system, this would be determined by actual grasp attempt

        if success:
            result.success = True
            result.status = 'grasp_successful'
        else:
            result.success = False
            result.status = 'grasp_failed'

        result.timestamp = self.get_clock().now().to_msg()
        result.grasp_force_applied = 10.0  # Simulated force

        return result


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()