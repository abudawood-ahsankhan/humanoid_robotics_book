#!/usr/bin/env python3

"""
Navigation Node for VLA System

This node handles navigation actions using ROS 2 navigation system.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from vla_msgs.msg import ExecutionStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Create subscriber for navigation goals
        self.nav_goal_sub = self.create_subscription(
            PoseStamped,
            '/vla/nav_goal',
            self.nav_goal_callback,
            10
        )

        # Create publisher for navigation feedback
        self.nav_feedback_pub = self.create_publisher(Path, '/vla/nav_feedback', 10)

        # Create publisher for execution status
        self.status_pub = self.create_publisher(ExecutionStatus, '/vla/execution_status', 10)

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Navigation state
        self.current_goal = None
        self.navigation_active = False

        self.get_logger().info('Navigation Node initialized')

    def nav_goal_callback(self, msg):
        """Handle incoming navigation goal"""
        self.get_logger().info(f'Received navigation goal: {msg.pose.position.x}, {msg.pose.position.y}')
        self.current_goal = msg

        # Send navigation goal to Nav2
        self.send_navigation_goal(msg)

    def send_navigation_goal(self, goal_pose):
        """Send navigation goal to Nav2 system"""
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )

        self._send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        self.navigation_active = True

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.current_pose}')

        # Publish execution status
        status_msg = ExecutionStatus()
        status_msg.plan_id = 'current_plan'  # Would be actual plan ID in real implementation
        status_msg.action_id = 'navigation_action'
        status_msg.status = 'in_progress'
        status_msg.progress = 0.5  # Placeholder progress
        status_msg.start_time = self.get_clock().now().to_msg()
        status_msg.feedback = [f'Navigating to goal, current position: {feedback.current_pose.pose.position}']

        self.status_pub.publish(status_msg)

    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.navigation_active = False

        # Publish execution status
        status_msg = ExecutionStatus()
        status_msg.plan_id = 'current_plan'
        status_msg.action_id = 'navigation_action'
        status_msg.status = 'completed' if result else 'failed'
        status_msg.progress = 1.0 if result else 0.0
        status_msg.start_time = self.get_clock().now().to_msg()
        status_msg.estimated_completion = self.get_clock().now().to_msg()
        status_msg.feedback = [f'Navigation completed with result: {result}']

        self.status_pub.publish(status_msg)

        if result:
            self.get_logger().info('Navigation completed successfully')
        else:
            self.get_logger().error('Navigation failed')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()