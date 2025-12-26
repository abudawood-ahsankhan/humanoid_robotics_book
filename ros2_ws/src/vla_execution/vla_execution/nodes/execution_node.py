#!/usr/bin/env python3

"""
Execution Node for VLA System

This node manages action execution and state tracking.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import ActionPlan, ExecutionStatus
from std_msgs.msg import String


class ExecutionNode(Node):
    def __init__(self):
        super().__init__('execution_node')

        # Create subscriber for action plans
        self.plan_sub = self.create_subscription(
            ActionPlan,
            '/vla/action_plan',
            self.plan_callback,
            10
        )

        # Create subscriber for execution status
        self.status_sub = self.create_subscription(
            ExecutionStatus,
            '/vla/execution_status',
            self.status_callback,
            10
        )

        # Create publisher for execution feedback
        self.feedback_pub = self.create_publisher(ExecutionStatus, '/vla/execution_feedback', 10)

        # Execution state
        self.current_plan = None
        self.current_action_index = 0
        self.execution_active = False

        self.get_logger().info('Execution Node initialized')

    def plan_callback(self, msg):
        """Handle incoming action plan"""
        self.get_logger().info(f'Received action plan with {len(msg.actions)} actions')
        self.current_plan = msg
        self.current_action_index = 0
        self.execution_active = True

        # Start execution of the plan
        self.execute_plan()

    def status_callback(self, msg):
        """Handle execution status updates"""
        self.get_logger().info(f'Execution status: {msg.status} for action {msg.action_id}')

        # Update our internal state based on the status
        if msg.status == 'completed':
            self.current_action_index += 1
            if self.current_action_index < len(self.current_plan.actions):
                # Execute the next action
                self.execute_next_action()
            else:
                # Plan completed
                self.plan_completed()
        elif msg.status == 'failed':
            self.plan_failed(msg)

    def execute_plan(self):
        """Start executing the current plan"""
        if self.current_plan and len(self.current_plan.actions) > 0:
            self.get_logger().info(f'Starting execution of plan {self.current_plan.plan_id}')
            self.execute_next_action()

    def execute_next_action(self):
        """Execute the next action in the plan"""
        if (self.current_plan and
            self.current_action_index < len(self.current_plan.actions)):

            action = self.current_plan.actions[self.current_action_index]
            self.get_logger().info(f'Executing action: {action.description}')

            # Publish execution status
            status_msg = ExecutionStatus()
            status_msg.plan_id = self.current_plan.plan_id
            status_msg.action_id = action.action_id
            status_msg.status = 'in_progress'
            status_msg.progress = 0.0
            status_msg.start_time = self.get_clock().now().to_msg()
            status_msg.feedback = [f'Starting execution of: {action.description}']

            self.feedback_pub.publish(status_msg)

    def plan_completed(self):
        """Handle plan completion"""
        self.get_logger().info(f'Plan {self.current_plan.plan_id} completed successfully')
        self.execution_active = False

        # Publish completion status
        status_msg = ExecutionStatus()
        status_msg.plan_id = self.current_plan.plan_id
        status_msg.action_id = 'all'
        status_msg.status = 'completed'
        status_msg.progress = 1.0
        status_msg.start_time = self.get_clock().now().to_msg()
        status_msg.estimated_completion = self.get_clock().now().to_msg()
        status_msg.feedback = ['Plan completed successfully']

        self.feedback_pub.publish(status_msg)

    def plan_failed(self, status_msg):
        """Handle plan failure"""
        self.get_logger().error(f'Plan failed at action {status_msg.action_id}')
        self.execution_active = False

        # Publish failure status
        failure_msg = ExecutionStatus()
        failure_msg.plan_id = self.current_plan.plan_id
        failure_msg.action_id = status_msg.action_id
        failure_msg.status = 'failed'
        failure_msg.progress = status_msg.progress
        failure_msg.start_time = status_msg.start_time
        failure_msg.estimated_completion = self.get_clock().now().to_msg()
        failure_msg.feedback = [f'Plan failed at action {status_msg.action_id}: {status_msg.feedback}']

        self.feedback_pub.publish(failure_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExecutionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()