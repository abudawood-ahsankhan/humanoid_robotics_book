#!/usr/bin/env python3
"""
Localization Accuracy Test for VSLAM System

This script tests the localization accuracy of the VSLAM system to ensure
it meets the <10cm accuracy requirement.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
import numpy as np
import time
import threading
from collections import deque
import math


class LocalizationAccuracyTest(Node):
    def __init__(self):
        super().__init__('localization_accuracy_test')

        # Declare parameters
        self.declare_parameter('test_duration', 120.0)  # 2 minutes
        self.declare_parameter('vslam_pose_topic', '/vslam/pose')
        self.declare_parameter('ground_truth_topic', '/ground_truth/pose')
        self.declare_parameter('max_localization_error', 0.1)  # 10cm
        self.declare_parameter('error_threshold', 0.1)  # 10cm threshold
        self.declare_parameter('sample_frequency', 10.0)  # Hz sampling
        self.declare_parameter('success_threshold', 0.9)  # 90% of samples under threshold

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.vslam_pose_topic = self.get_parameter('vslam_pose_topic').value
        self.ground_truth_topic = self.get_parameter('ground_truth_topic').value
        self.max_localization_error = self.get_parameter('max_localization_error').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.sample_frequency = self.get_parameter('sample_frequency').value
        self.success_threshold = self.get_parameter('success_threshold').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize test variables
        self.test_start_time = None
        self.test_active = False
        self.localization_errors = deque(maxlen=1000)
        self.position_errors = deque(maxlen=1000)
        self.orientation_errors = deque(maxlen=1000)
        self.data_lock = threading.Lock()

        # Ground truth and estimated poses
        self.ground_truth_pose = None
        self.vslam_pose = None
        self.last_comparison_time = None

        # Test results
        self.test_results = {
            'mean_position_error': 0.0,
            'max_position_error': 0.0,
            'median_position_error': 0.0,
            'std_position_error': 0.0,
            'success_rate': 0.0,
            'total_samples': 0,
            'passed': False
        }

        # Create subscribers
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped,
            self.vslam_pose_topic,
            self.vslam_pose_callback,
            sensor_qos
        )

        self.ground_truth_sub = self.create_subscription(
            PoseStamped,
            self.ground_truth_topic,
            self.ground_truth_callback,
            sensor_qos
        )

        self.get_logger().info('Localization Accuracy Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Error threshold: {self.error_threshold}m')
        self.get_logger().info(f'Success threshold: {self.success_threshold * 100}%')

    def vslam_pose_callback(self, msg):
        """Callback for VSLAM pose messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.vslam_pose = {
                'position': np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                'orientation': np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w]),
                'timestamp': msg.header.stamp
            }

        # Calculate error if we have ground truth
        if self.ground_truth_pose is not None:
            self.calculate_localization_error()

    def ground_truth_callback(self, msg):
        """Callback for ground truth pose messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.ground_truth_pose = {
                'position': np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                'orientation': np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w]),
                'timestamp': msg.header.stamp
            }

        # Calculate error if we have VSLAM pose
        if self.vslam_pose is not None:
            self.calculate_localization_error()

    def calculate_localization_error(self):
        """Calculate localization error between VSLAM and ground truth"""
        if self.vslam_pose is None or self.ground_truth_pose is None:
            return

        # Calculate position error (Euclidean distance)
        position_error = np.linalg.norm(
            self.vslam_pose['position'] - self.ground_truth_pose['position']
        )

        # Calculate orientation error (angle between quaternions)
        q1 = self.vslam_pose['orientation']
        q2 = self.ground_truth_pose['orientation']
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        # Calculate dot product
        dot_product = np.abs(np.dot(q1, q2))
        # Calculate angle error (in radians)
        orientation_error = 2 * np.arccos(np.clip(dot_product, 0.0, 1.0))

        # Store errors
        with self.data_lock:
            self.position_errors.append(position_error)
            self.orientation_errors.append(orientation_error)
            self.localization_errors.append(position_error)  # Using position error for main metric

        # Log error if it's above threshold
        if position_error > self.error_threshold:
            self.get_logger().warn(f'High localization error: {position_error:.3f}m')

    def start_test(self):
        """Start the localization accuracy test"""
        self.get_logger().info('Starting localization accuracy test...')
        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Wait for test duration
        start_time = time.time()
        while time.time() - start_time < self.test_duration and self.test_active:
            time.sleep(0.1)

        # Stop test
        self.test_active = False
        self.get_logger().info('Localization accuracy test completed')

        # Process results
        self.process_test_results()

        # Generate report
        self.generate_test_report()

    def process_test_results(self):
        """Process collected data to calculate test metrics"""
        with self.data_lock:
            if len(self.position_errors) == 0:
                self.get_logger().warn('No localization data collected during test')
                return

            # Calculate position error metrics
            position_errors_array = np.array(list(self.position_errors))
            self.test_results['mean_position_error'] = float(np.mean(position_errors_array))
            self.test_results['max_position_error'] = float(np.max(position_errors_array))
            self.test_results['median_position_error'] = float(np.median(position_errors_array))
            self.test_results['std_position_error'] = float(np.std(position_errors_array))
            self.test_results['total_samples'] = len(position_errors_array)

            # Calculate success rate (percentage of samples under threshold)
            errors_under_threshold = sum(1 for err in position_errors_array if err <= self.error_threshold)
            self.test_results['success_rate'] = errors_under_threshold / len(position_errors_array) if len(position_errors_array) > 0 else 0
            self.test_results['passed'] = self.test_results['success_rate'] >= self.success_threshold

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== LOCALIZATION ACCURACY TEST REPORT ===')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')

        # Print individual metrics
        self.get_logger().info(f'--- LOCALIZATION METRICS ---')
        self.get_logger().info(f'Mean Position Error: {self.test_results["mean_position_error"]:.3f}m')
        self.get_logger().info(f'Max Position Error: {self.test_results["max_position_error"]:.3f}m')
        self.get_logger().info(f'Median Position Error: {self.test_results["median_position_error"]:.3f}m')
        self.get_logger().info(f'Std Position Error: {self.test_results["std_position_error"]:.3f}m')
        self.get_logger().info(f'Total Samples: {self.test_results["total_samples"]}')
        self.get_logger().info(f'Success Rate: {self.test_results["success_rate"]:.3f} ({self.test_results["success_rate"] * 100:.1f}%)')

        # Print requirements
        self.get_logger().info(f'--- REQUIREMENTS ---')
        self.get_logger().info(f'Error Threshold: {self.error_threshold}m')
        self.get_logger().info(f'Success Threshold: {self.success_threshold * 100:.1f}%')

        # Determine pass/fail
        mean_error_pass = self.test_results['mean_position_error'] <= self.error_threshold
        max_error_pass = self.test_results['max_position_error'] <= self.error_threshold
        success_rate_pass = self.test_results['success_rate'] >= self.success_threshold

        self.get_logger().info(f'--- PASS/FAIL CRITERIA ---')
        self.get_logger().info(f'Mean Error (≤{self.error_threshold}m): {"✓ PASS" if mean_error_pass else "✗ FAIL"}')
        self.get_logger().info(f'Max Error (≤{self.error_threshold}m): {"✓ PASS" if max_error_pass else "✗ FAIL"}')
        self.get_logger().info(f'Success Rate (≥{self.success_threshold * 100:.0f}%): {"✓ PASS" if success_rate_pass else "✗ FAIL"}')

        # Overall assessment
        self.get_logger().info(f'--- FINAL ASSESSMENT ---')
        self.get_logger().info(f'Localization accuracy test: {"✓ PASS" if self.test_results["passed"] else "✗ FAIL"}')

        # Check if <10cm requirement is met
        ten_cm_requirement = self.test_results['mean_position_error'] < 0.1
        self.get_logger().info(f'10cm requirement (<0.1m): {"✓ PASS" if ten_cm_requirement else "✗ FAIL"}')

        # Recommendations
        self.get_logger().info(f'--- RECOMMENDATIONS ---')
        if not mean_error_pass:
            self.get_logger().info('  - Improve camera calibration')
            self.get_logger().info('  - Enhance feature detection/tracking')
            self.get_logger().info('  - Optimize pose graph optimization')
        if not success_rate_pass:
            self.get_logger().info('  - Improve loop closure detection')
            self.get_logger().info('  - Adjust VSLAM parameters')
            self.get_logger().info('  - Consider sensor fusion with IMU')
        if not ten_cm_requirement:
            self.get_logger().info('  - Critical: Accuracy does not meet 10cm requirement')
            self.get_logger().info('  - Consider using additional sensors (LiDAR, wheel encoders)')
            self.get_logger().info('  - Implement more sophisticated sensor fusion')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = LocalizationAccuracyTest()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 10.0)  # Add 10s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('Localization accuracy test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()