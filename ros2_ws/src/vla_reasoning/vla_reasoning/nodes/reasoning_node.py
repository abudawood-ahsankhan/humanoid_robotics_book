#!/usr/bin/env python3

"""
Reasoning Node for VLA System

This node implements task decomposition for complex commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ActionPlan
from vla_msgs.srv import GeneratePlan
import json
import uuid


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')

        # Create subscriber for command text
        self.command_sub = self.create_subscription(
            String,
            '/vla/command_text',
            self.command_callback,
            10
        )

        # Create publisher for action plans
        self.plan_pub = self.create_publisher(ActionPlan, '/vla/action_plan', 10)

        # Create service server for plan generation
        self.plan_srv = self.create_service(
            GeneratePlan,
            '/vla/generate_plan',
            self.generate_plan_callback
        )

        # Plan generation parameters
        self.max_plan_steps = 50
        self.plan_validation_enabled = True

        self.get_logger().info('Reasoning Node initialized')

    def command_callback(self, msg):
        """Process incoming command and generate action plan"""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')

        # Generate action plan from command
        plan = self.generate_action_plan(command)

        # Publish action plan
        self.plan_pub.publish(plan)

    def generate_plan_callback(self, request, response):
        """Handle plan generation service request"""
        try:
            # Generate action plan from command
            plan = self.generate_action_plan(request.command, request.context, request.constraints)

            # Set response
            response.success = True
            response.plan = plan
            response.error_message = ""
            response.confidence = 0.95

            self.get_logger().info(f'Generated plan with {len(plan.actions)} actions')

        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            response.success = False
            response.error_message = str(e)
            response.confidence = 0.0

        return response

    def generate_action_plan(self, command, context=None, constraints=None):
        """Generate action plan from command text"""
        if context is None:
            context = []
        if constraints is None:
            constraints = []

        # Create action plan message
        plan = ActionPlan()
        plan.command_text = command
        plan.plan_id = str(uuid.uuid4())
        plan.plan_version = 1
        plan.confidence = 0.95
        plan.created_time = self.get_clock().now().to_msg()
        plan.safety_constraints = constraints

        # Parse command and generate actions
        actions = self.parse_command_to_actions(command)

        # Validate actions
        if self.plan_validation_enabled:
            actions = self.validate_actions(actions)

        plan.actions = actions

        return plan

    def parse_command_to_actions(self, command):
        """Parse command text to a list of actions"""
        command_lower = command.lower()
        actions = []

        # Simple rule-based parsing (in real system, would use LLM)
        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract destination
            destination = self.extract_destination(command_lower)
            if destination:
                action = self.create_navigation_action(destination)
                actions.append(action)

        if 'pick up' in command_lower or 'grasp' in command_lower or 'take' in command_lower:
            # Extract object to pick up
            obj = self.extract_object(command_lower)
            if obj:
                nav_action = self.create_navigation_action(f'location of {obj}')
                grasp_action = self.create_grasp_action(obj)
                actions.extend([nav_action, grasp_action])

        if 'place' in command_lower or 'put' in command_lower:
            # Extract object and location
            obj = self.extract_object(command_lower)
            destination = self.extract_destination(command_lower)
            if obj and destination:
                nav_action = self.create_navigation_action(destination)
                place_action = self.create_place_action(obj, destination)
                actions.extend([nav_action, place_action])

        # If no specific actions identified, create a default plan
        if not actions:
            action = self.create_default_action(command)
            actions.append(action)

        return actions

    def extract_destination(self, command):
        """Extract destination from command"""
        # Simple extraction - look for location words
        locations = ['kitchen', 'bedroom', 'living room', 'office', 'table', 'counter', 'bin']
        for loc in locations:
            if loc in command:
                return loc
        return 'destination'

    def extract_object(self, command):
        """Extract object from command"""
        # Simple extraction - look for object words
        objects = ['cup', 'ball', 'book', 'box', 'red ball', 'blue cup']
        for obj in objects:
            if obj in command:
                return obj
        return 'object'

    def create_navigation_action(self, destination):
        """Create a navigation action"""
        from vla_msgs.msg import Action, KeyValue
        action = Action()
        action.action_id = str(uuid.uuid4())
        action.action_type = 'navigation'
        action.description = f'Navigate to {destination}'
        action.priority = 1.0
        action.estimated_duration = 10.0

        # Add destination parameter
        dest_param = KeyValue()
        dest_param.key = 'destination'
        dest_param.value = destination
        action.parameters = [dest_param]

        return action

    def create_grasp_action(self, obj):
        """Create a grasp action"""
        from vla_msgs.msg import Action, KeyValue
        action = Action()
        action.action_id = str(uuid.uuid4())
        action.action_type = 'manipulation'
        action.description = f'Grasp {obj}'
        action.priority = 1.0
        action.estimated_duration = 5.0

        # Add object parameter
        obj_param = KeyValue()
        obj_param.key = 'object'
        obj_param.value = obj
        action.parameters = [obj_param]

        return action

    def create_place_action(self, obj, destination):
        """Create a place action"""
        from vla_msgs.msg import Action, KeyValue
        action = Action()
        action.action_id = str(uuid.uuid4())
        action.action_type = 'manipulation'
        action.description = f'Place {obj} at {destination}'
        action.priority = 1.0
        action.estimated_duration = 5.0

        # Add object and destination parameters
        obj_param = KeyValue()
        obj_param.key = 'object'
        obj_param.value = obj
        dest_param = KeyValue()
        dest_param.key = 'destination'
        dest_param.value = destination
        action.parameters = [obj_param, dest_param]

        return action

    def create_default_action(self, command):
        """Create a default action for unrecognized commands"""
        from vla_msgs.msg import Action
        action = Action()
        action.action_id = str(uuid.uuid4())
        action.action_type = 'communication'
        action.description = f'Unable to parse command: {command}'
        action.priority = 1.0
        action.estimated_duration = 1.0
        action.parameters = []

        return action

    def validate_actions(self, actions):
        """Validate generated actions"""
        # Basic validation - ensure all actions have required fields
        valid_actions = []
        for action in actions:
            if action.action_type and action.description:
                valid_actions.append(action)

        return valid_actions


def main(args=None):
    rclpy.init(args=args)
    node = ReasoningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()