#!/usr/bin/env python3

"""
VLA Coordinator Node for VLA System

This node coordinates the overall VLA system by managing data flow between layers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionPlan, ExecutionStatus
from vla_msgs.srv import ProcessVoice, GeneratePlan


class VLACoordinatorNode(Node):
    def __init__(self):
        super().__init__('vla_coordinator_node')

        # Create subscribers for various topics
        self.command_sub = self.create_subscription(
            String,
            '/vla/command_text',
            self.command_callback,
            10
        )

        self.plan_sub = self.create_subscription(
            ActionPlan,
            '/vla/action_plan',
            self.plan_callback,
            10
        )

        self.status_sub = self.create_subscription(
            ExecutionStatus,
            '/vla/execution_status',
            self.status_callback,
            10
        )

        # Create publishers for coordination
        self.status_pub = self.create_publisher(ExecutionStatus, '/vla/system_status', 10)

        # Create service clients
        self.process_voice_cli = self.create_client(ProcessVoice, '/vla/process_voice')
        self.generate_plan_cli = self.create_client(GeneratePlan, '/vla/generate_plan')

        # Create service servers
        self.text_command_srv = self.create_service(
            ProcessVoice,
            '/vla/text_command',
            self.text_command_callback
        )

        self.get_logger().info('VLA Coordinator Node initialized')

    def command_callback(self, msg):
        """Handle incoming command text"""
        self.get_logger().info(f'Received command: {msg.data}')
        # Forward command for plan generation
        # In a real implementation, this would call the reasoning node

    def plan_callback(self, msg):
        """Handle incoming action plan"""
        self.get_logger().info(f'Received action plan with {len(msg.actions)} actions')
        # Forward plan for execution

    def status_callback(self, msg):
        """Handle incoming execution status"""
        self.get_logger().info(f'Execution status: {msg.status}')

    def text_command_callback(self, request, response):
        """Handle text command service request"""
        self.get_logger().info(f'Text command received: {request}')
        # Process the command and generate a response
        response.success = True
        response.text = request.command
        response.confidence = 1.0
        response.error_message = ""
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VLACoordinatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()