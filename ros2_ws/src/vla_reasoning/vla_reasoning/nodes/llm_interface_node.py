#!/usr/bin/env python3

"""
LLM Interface Node for VLA System

This node interfaces with Large Language Models for task planning.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import PlanRequest, PlanResponse
from vla_msgs.srv import GeneratePlan
import openai
import json
import uuid
import os


class LLMInterfaceNode(Node):
    def __init__(self):
        super().__init__('llm_interface_node')

        # Declare parameters
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('api_key', '')
        self.declare_parameter('temperature', 0.3)
        self.declare_parameter('max_tokens', 1000)
        self.declare_parameter('timeout', 30.0)

        # Get parameters
        self.model = self.get_parameter('model').value
        self.api_key = self.get_parameter('api_key').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.timeout = self.get_parameter('timeout').value

        # Set OpenAI API key
        if self.api_key:
            openai.api_key = self.api_key
        elif 'OPENAI_API_KEY' in os.environ:
            openai.api_key = os.environ['OPENAI_API_KEY']
        else:
            self.get_logger().warning('No OpenAI API key provided. Using mock responses.')

        # Create subscriber for plan requests
        self.plan_request_sub = self.create_subscription(
            PlanRequest,
            '/vla/plan_request',
            self.plan_request_callback,
            10
        )

        # Create publisher for plan responses
        self.plan_response_pub = self.create_publisher(PlanResponse, '/vla/plan_response', 10)

        # Create service server for plan generation
        self.generate_plan_srv = self.create_service(
            GeneratePlan,
            '/vla/generate_plan',
            self.generate_plan_callback
        )

        self.get_logger().info('LLM Interface Node initialized')

    def plan_request_callback(self, msg):
        """Handle incoming plan requests"""
        self.get_logger().info(f'Received plan request: {msg.command}')
        # Process the request and generate a plan response
        # This would call the actual LLM processing

    def generate_plan_callback(self, request, response):
        """Handle plan generation service request"""
        try:
            # Generate plan using LLM
            plan = self.generate_plan_with_llm(request.command, request.context, request.constraints)

            # Create response
            response.success = True
            response.plan = plan
            response.error_message = ""
            response.confidence = 0.9

            # Publish plan response
            plan_response_msg = PlanResponse()
            plan_response_msg.request_id = str(uuid.uuid4())  # Could use request ID if available
            plan_response_msg.plan = plan
            plan_response_msg.success = True
            plan_response_msg.error_message = ""
            plan_response_msg.confidence = 0.9
            plan_response_msg.timestamp = self.get_clock().now().to_msg()

            self.plan_response_pub.publish(plan_response_msg)

            self.get_logger().info(f'Generated plan with LLM for command: {request.command}')

        except Exception as e:
            self.get_logger().error(f'Error generating plan with LLM: {e}')
            response.success = False
            response.error_message = str(e)
            response.confidence = 0.0

        return response

    def generate_plan_with_llm(self, command, context, constraints):
        """Generate action plan using LLM"""
        # Create a prompt for the LLM
        prompt = self.create_plan_prompt(command, context, constraints)

        # Use mock implementation if no API key is available
        if not openai.api_key:
            return self.generate_mock_plan(command)

        try:
            # Call the LLM API
            completion = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that converts natural language commands into structured action plans for a humanoid robot. Respond in JSON format."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )

            # Parse the response
            response_text = completion.choices[0].message['content']
            plan_data = json.loads(response_text)

            # Convert to ActionPlan message
            plan = self.convert_to_action_plan(plan_data, command)
            return plan

        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {e}')
            # Return mock plan if LLM call fails
            return self.generate_mock_plan(command)

    def create_plan_prompt(self, command, context, constraints):
        """Create a prompt for the LLM"""
        prompt = f"""
        Convert the following natural language command into a structured action plan for a humanoid robot:

        Command: "{command}"

        Context: {context}

        Constraints: {constraints}

        Please respond with a JSON object in the following format:
        {{
          "command_text": "original command",
          "plan_id": "unique identifier",
          "plan_version": 1,
          "actions": [
            {{
              "action_id": "unique action id",
              "action_type": "navigation | manipulation | perception | communication",
              "description": "detailed action description",
              "parameters": [
                {{"key": "param_name", "value": "param_value"}}
              ],
              "priority": 1.0,
              "estimated_duration": 10.0
            }}
          ],
          "confidence": 0.9,
          "safety_constraints": ["constraint1", "constraint2"]
        }}

        Make sure the actions are specific and executable by a robot.
        """

        return prompt

    def convert_to_action_plan(self, plan_data, original_command):
        """Convert LLM response to ActionPlan message"""
        from vla_msgs.msg import ActionPlan, Action, KeyValue
        import builtin_interfaces.msg
        import uuid

        plan = ActionPlan()
        plan.command_text = original_command
        plan.plan_id = plan_data.get('plan_id', str(uuid.uuid4()))
        plan.plan_version = plan_data.get('plan_version', 1)
        plan.confidence = plan_data.get('confidence', 0.9)
        plan.safety_constraints = plan_data.get('safety_constraints', [])
        plan.created_time = self.get_clock().now().to_msg()

        # Convert actions
        actions = []
        for action_data in plan_data.get('actions', []):
            action = Action()
            action.action_id = action_data.get('action_id', str(uuid.uuid4()))
            action.action_type = action_data.get('action_type', 'unknown')
            action.description = action_data.get('description', '')
            action.priority = action_data.get('priority', 1.0)
            action.estimated_duration = builtin_interfaces.msg.Duration(sec=int(action_data.get('estimated_duration', 10.0)))

            # Convert parameters
            key_values = []
            for param in action_data.get('parameters', []):
                kv = KeyValue()
                kv.key = param.get('key', '')
                kv.value = str(param.get('value', ''))
                key_values.append(kv)

            action.parameters = key_values
            actions.append(action)

        plan.actions = actions

        return plan

    def generate_mock_plan(self, command):
        """Generate a mock plan when LLM is not available"""
        from vla_msgs.msg import ActionPlan, Action, KeyValue
        import builtin_interfaces.msg
        import uuid

        # Create a simple mock plan based on the command
        plan = ActionPlan()
        plan.command_text = command
        plan.plan_id = str(uuid.uuid4())
        plan.plan_version = 1
        plan.confidence = 0.8
        plan.safety_constraints = ["avoid_obstacles", "respect_personal_space"]
        plan.created_time = self.get_clock().now().to_msg()

        # Create a simple action based on command
        action = Action()
        action.action_id = str(uuid.uuid4())
        action.description = f"Execute command: {command}"

        # Determine action type based on command
        command_lower = command.lower()
        if any(word in command_lower for word in ['go', 'move', 'navigate', 'walk', 'drive']):
            action.action_type = 'navigation'
        elif any(word in command_lower for word in ['pick', 'grasp', 'take', 'put', 'place', 'lift', 'drop']):
            action.action_type = 'manipulation'
        elif any(word in command_lower for word in ['find', 'look', 'see', 'detect', 'identify']):
            action.action_type = 'perception'
        else:
            action.action_type = 'communication'

        action.priority = 1.0
        action.estimated_duration = builtin_interfaces.msg.Duration(sec=10)

        # Add mock parameters
        param = KeyValue()
        param.key = 'command'
        param.value = command
        action.parameters = [param]

        plan.actions = [action]

        return plan


def main(args=None):
    rclpy.init(args=args)
    node = LLMInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()