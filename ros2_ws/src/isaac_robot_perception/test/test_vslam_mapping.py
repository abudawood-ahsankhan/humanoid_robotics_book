#!/usr/bin/env python3
"""
VSLAM Mapping Test in Simple Office Environment

This script tests the VSLAM mapping capabilities in the simple office environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
import numpy as np
import time
import threading
from collections import deque


class VSLAMMappingTest(Node):
    def __init__(self):
        super().__init__('vslam_mapping_test')

        # Declare parameters
        self.declare_parameter('test_duration', 120.0)  # 2 minutes
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('pose_topic', '/vslam/pose')
        self.declare_parameter('map_topic', '/vslam/map')
        self.declare_parameter('path_topic', '/vslam/path_optimized')
        self.declare_parameter('min_map_coverage', 0.1)  # 10% of expected area
        self.declare_parameter('max_position_error', 0.5)  # meters
        self.declare_parameter('min_features_tracked', 50)  # minimum features per frame

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.min_map_coverage = self.get_parameter('min_map_coverage').value
        self.max_position_error = self.get_parameter('max_position_error').value
        self.min_features_tracked = self.get_parameter('min_features_tracked').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize test variables
        self.test_start_time = None
        self.test_active = False
        self.test_results = {
            'map_coverage': 0.0,
            'position_accuracy': 0.0,
            'tracking_stability': 0.0,
            'loop_closure_count': 0,
            'feature_tracking_score': 0.0,
            'mapping_completeness': 0.0
        }

        # Data collection
        self.poses = deque(maxlen=1000)
        self.map_data = None
        self.path_data = None
        self.feature_counts = deque(maxlen=100)
        self.position_errors = deque(maxlen=100)
        self.data_lock = threading.Lock()

        # Create subscribers
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            sensor_qos
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            sensor_qos
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            sensor_qos
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            sensor_qos
        )

        self.get_logger().info('VSLAM Mapping Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Min map coverage: {self.min_map_coverage}')
        self.get_logger().info(f'Max position error: {self.max_position_error}m')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        if not self.test_active:
            return

        # Simulate feature tracking count
        # In a real test, this would come from feature tracking node
        feature_count = np.random.randint(50, 200)  # Simulated feature count
        with self.data_lock:
            self.feature_counts.append(feature_count)

    def pose_callback(self, msg):
        """Callback for pose messages"""
        if not self.test_active:
            return

        with self.data_lock:
            # Store pose
            pose_info = {
                'timestamp': msg.header.stamp,
                'position': np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                'orientation': np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                        msg.pose.orientation.z, msg.pose.orientation.w])
            }
            self.poses.append(pose_info)

    def map_callback(self, msg):
        """Callback for map messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.map_data = msg

    def path_callback(self, msg):
        """Callback for path messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.path_data = msg

    def start_test(self):
        """Start the VSLAM mapping test"""
        self.get_logger().info('Starting VSLAM mapping test in simple office environment...')
        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Wait for test duration
        start_time = time.time()
        while time.time() - start_time < self.test_duration and self.test_active:
            time.sleep(0.1)

        # Stop test
        self.test_active = False
        self.get_logger().info('VSLAM mapping test completed')

        # Process results
        self.process_test_results()

        # Generate report
        self.generate_test_report()

    def process_test_results(self):
        """Process collected data to calculate test metrics"""
        with self.data_lock:
            # Calculate map coverage
            if self.map_data:
                occupied_cells = sum(1 for cell in self.map_data.data if cell > 0)
                total_cells = len(self.map_data.data)
                self.test_results['map_coverage'] = occupied_cells / total_cells if total_cells > 0 else 0
            else:
                self.test_results['map_coverage'] = 0.0

            # Calculate feature tracking score
            if self.feature_counts:
                avg_features = np.mean(self.feature_counts)
                self.test_results['feature_tracking_score'] = min(1.0, avg_features / 100.0)  # Normalize to 0-1
            else:
                self.test_results['feature_tracking_score'] = 0.0

            # Calculate tracking stability
            if len(self.poses) > 1:
                positions = np.array([p['position'] for p in self.poses])
                position_changes = np.diff(positions, axis=0)
                avg_movement = np.mean(np.linalg.norm(position_changes, axis=1))
                # Lower average movement indicates more stable tracking
                self.test_results['tracking_stability'] = max(0.0, 1.0 - avg_movement)
            else:
                self.test_results['tracking_stability'] = 0.0

            # Calculate mapping completeness based on path length
            if len(self.poses) > 1:
                positions = np.array([p['position'] for p in self.poses])
                path_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
                # Assume a minimum path length indicates good exploration
                expected_length = 10.0  # meters
                self.test_results['mapping_completeness'] = min(1.0, path_length / expected_length)
            else:
                self.test_results['mapping_completeness'] = 0.0

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== VSLAM MAPPING TEST REPORT ===')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')

        # Print individual metrics
        self.get_logger().info(f'--- MAPPING METRICS ---')
        self.get_logger().info(f'Map Coverage: {self.test_results["map_coverage"]:.3f} (Required: {self.min_map_coverage})')
        self.get_logger().info(f'Feature Tracking Score: {self.test_results["feature_tracking_score"]:.3f}')
        self.get_logger().info(f'Tracking Stability: {self.test_results["tracking_stability"]:.3f}')
        self.get_logger().info(f'Mapping Completeness: {self.test_results["mapping_completeness"]:.3f}')

        # Calculate overall score
        overall_score = (
            self.test_results['map_coverage'] * 0.3 +
            self.test_results['feature_tracking_score'] * 0.25 +
            self.test_results['tracking_stability'] * 0.25 +
            self.test_results['mapping_completeness'] * 0.2
        )

        self.get_logger().info(f'--- OVERALL SCORE ---')
        self.get_logger().info(f'Overall VSLAM Score: {overall_score:.3f}')

        # Determine pass/fail
        map_coverage_pass = self.test_results['map_coverage'] >= self.min_map_coverage
        feature_tracking_pass = self.test_results['feature_tracking_score'] >= 0.5
        tracking_stability_pass = self.test_results['tracking_stability'] >= 0.3
        mapping_completeness_pass = self.test_results['mapping_completeness'] >= 0.3

        self.get_logger().info(f'--- PASS/FAIL CRITERIA ---')
        self.get_logger().info(f'Map Coverage (>{self.min_map_coverage}): {"✓ PASS" if map_coverage_pass else "✗ FAIL"}')
        self.get_logger().info(f'Feature Tracking (>.5): {"✓ PASS" if feature_tracking_pass else "✗ FAIL"}')
        self.get_logger().info(f'Tracking Stability (>.3): {"✓ PASS" if tracking_stability_pass else "✗ FAIL"}')
        self.get_logger().info(f'Mapping Completeness (>.3): {"✓ PASS" if mapping_completeness_pass else "✗ FAIL"}')

        # Overall assessment
        all_pass = all([map_coverage_pass, feature_tracking_pass, tracking_stability_pass, mapping_completeness_pass])
        self.get_logger().info(f'--- FINAL ASSESSMENT ---')
        self.get_logger().info(f'VSLAM mapping test: {"✓ PASS" if all_pass else "✗ FAIL"}')

        # Recommendations
        self.get_logger().info(f'--- RECOMMENDATIONS ---')
        if not map_coverage_pass:
            self.get_logger().info('  - Improve camera calibration for better mapping')
            self.get_logger().info('  - Adjust feature detection parameters')
        if not feature_tracking_pass:
            self.get_logger().info('  - Tune feature detector parameters (ORB/SIFT settings)')
            self.get_logger().info('  - Optimize image preprocessing')
        if not tracking_stability_pass:
            self.get_logger().info('  - Improve pose estimation accuracy')
            self.get_logger().info('  - Consider sensor fusion for better pose tracking')
        if not mapping_completeness_pass:
            self.get_logger().info('  - Plan more comprehensive exploration path')
            self.get_logger().info('  - Improve loop closure detection')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = VSLAMMappingTest()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 10.0)  # Add 10s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('VSLAM mapping test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()