#!/usr/bin/env python3

"""
Plan Validator Node for VLA System

This node validates LLM outputs against safety constraints.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import ActionPlan
from vla_msgs.srv import GeneratePlan
from vla_msgs.msg import SafetyStatus


class PlanValidatorNode(Node):
    def __init__(self):
        super().__init__('plan_validator_node')

        # Create subscriber for action plans
        self.plan_sub = self.create_subscription(
            ActionPlan,
            '/vla/action_plan',
            self.plan_callback,
            10
        )

        # Create publisher for safety status
        self.safety_pub = self.create_publisher(SafetyStatus, '/vla/safety_status', 10)

        # Safety validation parameters
        self.safety_validation_enabled = True
        self.human_in_the_loop_enabled = True

        self.get_logger().info('Plan Validator Node initialized')

    def plan_callback(self, msg):
        """Validate incoming action plan"""
        self.get_logger().info(f'Validating action plan with {len(msg.actions)} actions')

        # Validate the plan
        validation_result = self.validate_plan(msg)

        if validation_result['safe']:
            self.get_logger().info('Plan validation passed')
            # Forward plan for execution
        else:
            self.get_logger().error(f'Plan validation failed: {validation_result["violations"]}')
            # Handle unsafe plan (reject, modify, or request human approval)

    def validate_plan(self, plan):
        """Validate an action plan against safety constraints"""
        result = {
            'safe': True,
            'violations': [],
            'confidence': 0.9
        }

        if not self.safety_validation_enabled:
            return result

        # Check each action in the plan
        for i, action in enumerate(plan.actions):
            action_violations = self.validate_action(action, i)
            result['violations'].extend(action_violations)

        # Check overall plan constraints
        plan_violations = self.validate_plan_constraints(plan)
        result['violations'].extend(plan_violations)

        # Update safety status
        safety_status = SafetyStatus()
        safety_status.plan_id = plan.plan_id
        safety_status.safe = len(result['violations']) == 0
        safety_status.violations = [v['description'] for v in result['violations']]
        safety_status.confidence = result['confidence']
        safety_status.timestamp = self.get_clock().now().to_msg()

        self.safety_pub.publish(safety_status)

        result['safe'] = len(result['violations']) == 0

        return result

    def validate_action(self, action, index):
        """Validate a single action"""
        violations = []

        # Check for valid action type
        valid_types = ['navigation', 'manipulation', 'perception', 'communication']
        if action.action_type not in valid_types:
            violations.append({
                'type': 'INVALID_ACTION_TYPE',
                'description': f'Action {index} has invalid type: {action.action_type}',
                'severity': 'ERROR'
            })

        # Check for empty description
        if not action.description.strip():
            violations.append({
                'type': 'EMPTY_DESCRIPTION',
                'description': f'Action {index} has empty description',
                'severity': 'WARNING'
            })

        # Check for reasonable duration
        if action.estimated_duration.sec > 3600:  # More than 1 hour
            violations.append({
                'type': 'EXCESSIVE_DURATION',
                'description': f'Action {index} has excessive duration: {action.estimated_duration.sec}s',
                'severity': 'WARNING'
            })

        # Check navigation-specific constraints
        if action.action_type == 'navigation':
            for param in action.parameters:
                if param.key == 'destination':
                    # Check if destination is in restricted areas
                    if self.is_restricted_area(param.value):
                        violations.append({
                            'type': 'RESTRICTED_AREA',
                            'description': f'Navigation action {index} targets restricted area: {param.value}',
                            'severity': 'ERROR'
                        })

        # Check manipulation-specific constraints
        if action.action_type == 'manipulation':
            for param in action.parameters:
                if param.key == 'object':
                    # Check if object is fragile or dangerous
                    if self.is_fragile_object(param.value):
                        violations.append({
                            'type': 'FRAGILE_OBJECT',
                            'description': f'Manipulation action {index} involves fragile object: {param.value}',
                            'severity': 'WARNING'
                        })

        return violations

    def validate_plan_constraints(self, plan):
        """Validate overall plan constraints"""
        violations = []

        # Check plan length
        if len(plan.actions) > 50:
            violations.append({
                'type': 'EXCESSIVE_PLAN_LENGTH',
                'description': f'Plan has {len(plan.actions)} actions, exceeding limit of 50',
                'severity': 'WARNING'
            })

        # Check safety constraints
        for constraint in plan.safety_constraints:
            if constraint == 'invalid_constraint':
                violations.append({
                    'type': 'INVALID_SAFETY_CONSTRAINT',
                    'description': f'Plan contains invalid safety constraint: {constraint}',
                    'severity': 'ERROR'
                })

        return violations

    def is_restricted_area(self, area):
        """Check if an area is restricted"""
        # In a real system, this would check against a map of restricted areas
        restricted_areas = ['server_room', 'emergency_exit', 'restricted_zone']
        return area.lower() in restricted_areas

    def is_fragile_object(self, obj):
        """Check if an object is fragile"""
        # In a real system, this would check against a database of object properties
        fragile_objects = ['glass', 'ceramic', 'vase', 'mirror', 'screen']
        return any(fragile in obj.lower() for fragile in fragile_objects)


def main(args=None):
    rclpy.init(args=args)
    node = PlanValidatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()