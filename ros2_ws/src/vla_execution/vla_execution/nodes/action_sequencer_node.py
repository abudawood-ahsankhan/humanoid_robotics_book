#!/usr/bin/env python3

"""
Action Sequencer Node for VLA System

This node handles action sequencing and coordination.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import ActionPlan, ExecutionStatus
from vla_msgs.action import ExecuteAction
from rclpy.action import ActionServer, GoalResponse, CancelResponse


class ActionSequencerNode(Node):
    def __init__(self):
        super().__init__('action_sequencer_node')

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

        # Create publisher for execution status
        self.status_pub = self.create_publisher(ExecutionStatus, '/vla/execution_status', 10)

        # Create action server for executing actions
        self.execute_action_server = ActionServer(
            self,
            ExecuteAction,
            'vla_execute',
            execute_callback=self.execute_action_callback,
            goal_callback=self.action_goal_callback,
            cancel_callback=self.action_cancel_callback
        )

        # Sequencer state
        self.current_plan = None
        self.current_action_index = 0
        self.sequencer_active = False
        self.action_dependencies = {}  # To track action dependencies

        self.get_logger().info('Action Sequencer Node initialized')

    def plan_callback(self, msg):
        """Handle incoming action plan"""
        self.get_logger().info(f'Received action plan with {len(msg.actions)} actions')
        self.current_plan = msg
        self.current_action_index = 0
        self.sequencer_active = True

        # Start executing the plan
        self.execute_plan()

    def status_callback(self, msg):
        """Handle execution status updates"""
        self.get_logger().info(f'Received status update: {msg.status} for action {msg.action_id}')

        # Check if we need to execute the next action based on dependencies
        if msg.status == 'completed':
            self.check_and_execute_next_action()

    def execute_plan(self):
        """Execute the current action plan"""
        if self.current_plan and len(self.current_plan.actions) > 0:
            self.get_logger().info(f'Starting execution of plan {self.current_plan.plan_id}')
            self.execute_next_action()

    def execute_next_action(self):
        """Execute the next action in the sequence"""
        if (self.current_plan and
            self.current_action_index < len(self.current_plan.actions)):

            action = self.current_plan.actions[self.current_action_index]
            self.get_logger().info(f'Executing action: {action.description}')

            # Execute the action via the action server
            self.execute_action(action)

    def check_and_execute_next_action(self):
        """Check if the next action can be executed based on dependencies"""
        # For now, we'll just execute actions sequentially
        # In a more complex system, we would check dependencies
        self.current_action_index += 1
        if self.current_action_index < len(self.current_plan.actions):
            self.execute_next_action()

    def execute_action_callback(self, goal_handle):
        """Execute action callback for the action server"""
        self.get_logger().info(f'Executing action: {goal_handle.request.action_type}')
        feedback_msg = ExecuteAction.Feedback()
        result_msg = ExecuteAction.Result()

        # Publish initial feedback
        feedback_msg.status = 'initializing'
        feedback_msg.progress = 0.0
        feedback_msg.current_step = 'Initializing action'
        goal_handle.publish_feedback(feedback_msg)

        # Simulate action execution based on action type
        action_type = goal_handle.request.action_type

        if action_type == 'navigation':
            feedback_msg.status = 'navigating'
            feedback_msg.progress = 0.3
            feedback_msg.current_step = 'Executing navigation'
            goal_handle.publish_feedback(feedback_msg)
            # Simulate navigation completion

        elif action_type == 'manipulation':
            feedback_msg.status = 'manipulating'
            feedback_msg.progress = 0.4
            feedback_msg.current_step = 'Executing manipulation'
            goal_handle.publish_feedback(feedback_msg)
            # Simulate manipulation completion

        elif action_type == 'perception':
            feedback_msg.status = 'perceiving'
            feedback_msg.progress = 0.5
            feedback_msg.current_step = 'Executing perception'
            goal_handle.publish_feedback(feedback_msg)
            # Simulate perception completion

        else:
            feedback_msg.status = 'executing'
            feedback_msg.progress = 0.2
            feedback_msg.current_step = 'Executing unknown action type'
            goal_handle.publish_feedback(feedback_msg)

        # Complete the action
        feedback_msg.status = 'completed'
        feedback_msg.progress = 1.0
        feedback_msg.current_step = 'Action completed'
        goal_handle.publish_feedback(feedback_msg)

        result_msg.success = True
        result_msg.result_message = f'{action_type} action completed successfully'
        result_msg.execution_time = 5.0  # Simulated execution time

        goal_handle.succeed()
        return result_msg

    def action_goal_callback(self, goal_request):
        """Handle action goal request"""
        self.get_logger().info('Received action goal request')
        return GoalResponse.ACCEPT

    def action_cancel_callback(self, goal_handle):
        """Handle action cancel request"""
        self.get_logger().info('Received action cancel request')
        return CancelResponse.ACCEPT

    def execute_action(self, action):
        """Execute a specific action"""
        # Create action goal
        from vla_msgs.action import ExecuteAction
        goal = ExecuteAction.Goal()
        goal.action_type = action.action_type

        # Add parameters to the goal
        for param in action.parameters:
            # Convert parameters as needed
            pass

        # In a real implementation, we would send this as an action goal
        # For now, we'll just simulate execution

        # Publish status that action is starting
        status_msg = ExecutionStatus()
        status_msg.plan_id = self.current_plan.plan_id if self.current_plan else 'unknown'
        status_msg.action_id = action.action_id
        status_msg.status = 'in_progress'
        status_msg.progress = 0.0
        status_msg.start_time = self.get_clock().now().to_msg()
        status_msg.feedback = [f'Starting execution of: {action.description}']

        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionSequencerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()