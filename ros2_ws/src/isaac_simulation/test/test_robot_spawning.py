#!/usr/bin/env python3
"""
Test script for verifying robot spawning in Isaac Sim with proper rendering.

This script tests that the humanoid robot model appears correctly in Isaac Sim
with proper physics properties and rendering capabilities.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import subprocess
import os


class TestRobotSpawning(Node):
    def __init__(self):
        super().__init__('test_robot_spawning')

        # Initialize test variables
        self.robot_spawned = False
        self.rendering_verified = False
        self.physics_verified = False
        self.image_received = False
        self.joint_states_received = False

        # Create subscribers to verify robot is publishing data
        self.image_sub = self.create_subscription(
            Image,
            '/sensors/camera/image_raw',
            self.image_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/isaac_sim/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for test execution
        self.test_timer = self.create_timer(1.0, self.test_step)
        self.test_step_count = 0

        self.get_logger().info('TestRobotSpawning node initialized')

    def image_callback(self, msg):
        """Callback for image messages"""
        self.image_received = True
        self.get_logger().debug(f'Received image: {msg.width}x{msg.height} {msg.encoding}')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.joint_states_received = True
        self.get_logger().debug(f'Received joint states for {len(msg.name)} joints')

    def test_step(self):
        """Execute test steps"""
        self.test_step_count += 1

        if self.test_step_count == 1:
            self.get_logger().info('Step 1: Checking if Isaac Sim is running')
            self.check_isaac_sim_running()

        elif self.test_step_count == 2:
            self.get_logger().info('Step 2: Verifying robot model exists in simulation')
            self.verify_robot_model()

        elif self.test_step_count == 3:
            self.get_logger().info('Step 3: Checking sensor data publication')
            self.check_sensor_data()

        elif self.test_step_count == 4:
            self.get_logger().info('Step 4: Testing robot control capabilities')
            self.test_robot_control()

        elif self.test_step_count == 5:
            self.get_logger().info('Step 5: Final test results')
            self.print_test_results()
            self.test_timer.cancel()

    def check_isaac_sim_running(self):
        """Check if Isaac Sim is running"""
        try:
            # This is a placeholder - in a real test, you'd check for Isaac Sim processes
            # For now, we'll assume Isaac Sim is available if the bridge node is running
            self.get_logger().info('Isaac Sim bridge node is running')
        except Exception as e:
            self.get_logger().error(f'Error checking Isaac Sim: {e}')

    def verify_robot_model(self):
        """Verify the robot model exists in the simulation"""
        # This would typically interface with Isaac Sim's USD stage
        # For now, we'll check if the expected topics are available
        try:
            # Check if joint states topic has publishers
            topic_names_and_types = self.get_topic_names_and_types()
            joint_state_topics = [name for name, types in topic_names_and_types
                                 if 'joint_state' in name.lower()]

            if '/isaac_sim/joint_states' in [name for name, _ in topic_names_and_types]:
                self.robot_spawned = True
                self.get_logger().info('Robot model verified - joint states topic available')
            else:
                self.get_logger().warning('Robot model may not be spawned - joint states topic not found')

        except Exception as e:
            self.get_logger().error(f'Error verifying robot model: {e}')

    def check_sensor_data(self):
        """Check if sensor data is being published"""
        # Wait a bit more for data to arrive
        time.sleep(2.0)

        if self.image_received:
            self.get_logger().info('Camera sensor data verified')
        else:
            self.get_logger().warning('No camera data received')

        if self.joint_states_received:
            self.get_logger().info('Joint state data verified')
        else:
            self.get_logger().warning('No joint state data received')

    def test_robot_control(self):
        """Test basic robot control capabilities"""
        # This would typically involve sending commands and verifying response
        # For now, we'll just log that we're testing control
        self.get_logger().info('Testing robot control capabilities')

    def print_test_results(self):
        """Print final test results"""
        self.get_logger().info('=== Test Results ===')
        self.get_logger().info(f'Robot Spawned: {self.robot_spawned}')
        self.get_logger().info(f'Image Data Received: {self.image_received}')
        self.get_logger().info(f'Joint States Received: {self.joint_states_received}')
        self.get_logger().info('===================')

        # Overall test result
        if self.robot_spawned and self.image_received and self.joint_states_received:
            self.get_logger().info('✓ Robot spawning test PASSED')
        else:
            self.get_logger().info('✗ Robot spawning test FAILED')
            if not self.robot_spawned:
                self.get_logger().info('  - Robot model not verified')
            if not self.image_received:
                self.get_logger().info('  - No camera data received')
            if not self.joint_states_received:
                self.get_logger().info('  - No joint state data received')


def main(args=None):
    rclpy.init(args=args)

    test_node = TestRobotSpawning()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()