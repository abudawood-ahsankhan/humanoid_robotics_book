#!/usr/bin/env python3

"""
Safety Monitor Node for VLA System

This node monitors all action execution for safety violations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, LaserScan
from vla_msgs.msg import SafetyStatus, ExecutionStatus
from vla_msgs.srv import CheckSafety
from builtin_interfaces.msg import Time


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        # Create subscribers for monitoring
        self.execution_sub = self.create_subscription(
            ExecutionStatus,
            '/vla/execution_status',
            self.execution_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )

        # Create publishers for safety status
        self.safety_status_pub = self.create_publisher(SafetyStatus, '/vla/safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Create service server for safety checks
        self.safety_check_srv = self.create_service(
            CheckSafety,
            '/vla/check_safety',
            self.safety_check_callback
        )

        # Safety parameters
        self.safety_enabled = True
        self.emergency_stop_active = False

        # Create timer for continuous monitoring
        self.monitor_timer = self.create_timer(0.1, self.safety_monitor)

        self.get_logger().info('Safety Monitor Node initialized')

    def execution_callback(self, msg):
        """Monitor execution status for safety violations"""
        if self.safety_enabled and not self.emergency_stop_active:
            # Check if the execution status indicates a potential safety issue
            if msg.status == "failed" or msg.status == "error":
                self.get_logger().warning(f'Potential safety issue detected in execution: {msg.status}')
                # Could trigger additional safety checks here

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        if self.safety_enabled and not self.emergency_stop_active:
            # Check for joint limits or dangerous configurations
            for i, position in enumerate(msg.position):
                # Add joint limit checking here
                pass

    def laser_scan_callback(self, msg):
        """Monitor laser scan for obstacles"""
        if self.safety_enabled and not self.emergency_stop_active:
            # Check for obstacles in robot path
            min_distance = min(msg.ranges) if msg.ranges else float('inf')
            if min_distance < 0.5:  # 50cm threshold
                self.get_logger().warning('Obstacle detected too close, triggering emergency stop')
                self.trigger_emergency_stop()

    def safety_monitor(self):
        """Continuous safety monitoring"""
        if self.safety_enabled and not self.emergency_stop_active:
            # Perform continuous safety checks
            pass

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop_active = True
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        self.get_logger().error('EMERGENCY STOP ACTIVATED')

    def safety_check_callback(self, request, response):
        """Handle safety check service request"""
        # Check if the proposed action is safe
        response.safe = True  # Simplified for now
        response.violations = []
        response.error_message = ""
        response.confidence = 0.95

        # Add actual safety checking logic here based on the proposed action
        # For now, we just return that it's safe
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()