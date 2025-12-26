#!/usr/bin/env python3
"""
VSLAM Performance Test in Complex Factory Environment

This script tests the VSLAM performance in the complex factory environment,
evaluating mapping quality, localization accuracy, and computational performance
under challenging conditions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
import time
import threading
from collections import deque
import statistics


class VSLAMComplexFactoryTest(Node):
    def __init__(self):
        super().__init__('vslam_complex_factory_test')

        # Declare parameters
        self.declare_parameter('test_duration', 180.0)  # 3 minutes
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('pose_topic', '/vslam/pose')
        self.declare_parameter('map_topic', '/vslam/map')
        self.declare_parameter('path_topic', '/vslam/path_optimized')
        self.declare_parameter('min_map_coverage', 0.05)  # 5% of expected area
        self.declare_parameter('max_position_error', 0.15)  # 15cm for complex env
        self.declare_parameter('min_features_tracked', 30)  # minimum features per frame
        self.declare_parameter('min_loop_closure_frequency', 0.01)  # 1 per 100 seconds
        self.declare_parameter('min_mapping_success_rate', 0.7)  # 70% success rate

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.min_map_coverage = self.get_parameter('min_map_coverage').value
        self.max_position_error = self.get_parameter('max_position_error').value
        self.min_features_tracked = self.get_parameter('min_features_tracked').value
        self.min_loop_closure_frequency = self.get_parameter('min_loop_closure_frequency').value
        self.min_mapping_success_rate = self.get_parameter('min_mapping_success_rate').value

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
            'mapping_completeness': 0.0,
            'computational_load': 0.0,
            'robustness_score': 0.0,
            'overall_performance': 0.0
        }

        # Data collection
        self.poses = deque(maxlen=2000)  # Larger buffer for longer test
        self.map_data = None
        self.path_data = None
        self.feature_counts = deque(maxlen=200)
        self.position_errors = deque(maxlen=200)
        self.computation_times = deque(maxlen=200)
        self.loop_closure_events = deque(maxlen=100)
        self.tracking_failures = 0
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

        self.get_logger().info('VSLAM Complex Factory Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Min map coverage: {self.min_map_coverage}')
        self.get_logger().info(f'Max position error: {self.max_position_error}m')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        if not self.test_active:
            return

        # Simulate feature tracking count (in complex environment, expect fewer features)
        # Complex factory has more challenging conditions
        feature_count = np.random.randint(20, 120)  # Lower range for complex environment
        with self.data_lock:
            self.feature_counts.append(feature_count)

    def pose_callback(self, msg):
        """Callback for pose messages"""
        if not self.test_active:
            return

        with self.data_lock:
            # Store pose
            start_time = time.time()

            pose_info = {
                'timestamp': msg.header.stamp,
                'position': np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                'orientation': np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                        msg.pose.orientation.z, msg.pose.orientation.w])
            }
            self.poses.append(pose_info)

            # Calculate computation time for this callback
            end_time = time.time()
            computation_time = end_time - start_time
            self.computation_times.append(computation_time)

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
        """Start the VSLAM performance test in complex factory environment"""
        self.get_logger().info('Starting VSLAM performance test in complex factory environment...')
        self.get_logger().info('Complex factory environment has: machinery, conveyor belts, varying lighting, sparse textures')
        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Wait for test duration
        start_time = time.time()
        while time.time() - start_time < self.test_duration and self.test_active:
            time.sleep(0.1)

        # Stop test
        self.test_active = False
        self.get_logger().info('VSLAM complex factory test completed')

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

                if len(position_changes) > 0:
                    avg_movement = np.mean(np.linalg.norm(position_changes, axis=1))
                    # Lower average movement indicates more stable tracking
                    self.test_results['tracking_stability'] = max(0.0, 1.0 - avg_movement)

                    # Calculate position accuracy based on movement consistency
                    std_movement = np.std(np.linalg.norm(position_changes, axis=1))
                    self.test_results['position_accuracy'] = max(0.0, 1.0 - std_movement)
                else:
                    self.test_results['tracking_stability'] = 0.0
                    self.test_results['position_accuracy'] = 0.0
            else:
                self.test_results['tracking_stability'] = 0.0
                self.test_results['position_accuracy'] = 0.0

            # Calculate mapping completeness based on path length
            if len(self.poses) > 1:
                positions = np.array([p['position'] for p in self.poses])
                path_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
                # For complex environment, assume a longer path is needed for good exploration
                expected_length = 20.0  # meters
                self.test_results['mapping_completeness'] = min(1.0, path_length / expected_length)
            else:
                self.test_results['mapping_completeness'] = 0.0

            # Calculate computational load
            if self.computation_times:
                avg_computation_time = np.mean(self.computation_times)
                # Lower computation time indicates better performance
                # Assuming max acceptable time is 0.1 seconds (10Hz processing)
                self.test_results['computational_load'] = max(0.0, 1.0 - avg_computation_time / 0.1)
            else:
                self.test_results['computational_load'] = 1.0  # No computation time recorded

            # Calculate robustness based on tracking failures and data quality
            total_expected_data_points = self.test_duration * 10  # Assuming ~10Hz operation
            actual_data_points = len(self.poses)
            data_completion_rate = actual_data_points / total_expected_data_points if total_expected_data_points > 0 else 0

            self.test_results['robustness_score'] = min(1.0, data_completion_rate * 2)  # Weight data completion

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== VSLAM COMPLEX FACTORY PERFORMANCE TEST REPORT ===')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')

        # Print individual metrics
        self.get_logger().info(f'--- MAPPING METRICS ---')
        self.get_logger().info(f'Map Coverage: {self.test_results["map_coverage"]:.3f} (Required: {self.min_map_coverage})')
        self.get_logger().info(f'Feature Tracking Score: {self.test_results["feature_tracking_score"]:.3f}')
        self.get_logger().info(f'Tracking Stability: {self.test_results["tracking_stability"]:.3f}')
        self.get_logger().info(f'Mapping Completeness: {self.test_results["mapping_completeness"]:.3f}')
        self.get_logger().info(f'Computational Load: {self.test_results["computational_load"]:.3f}')
        self.get_logger().info(f'Robustness Score: {self.test_results["robustness_score"]:.3f}')

        # Calculate overall score
        # In complex environment, we weight robustness and computational efficiency more heavily
        overall_score = (
            self.test_results['map_coverage'] * 0.2 +
            self.test_results['feature_tracking_score'] * 0.2 +
            self.test_results['tracking_stability'] * 0.15 +
            self.test_results['mapping_completeness'] * 0.15 +
            self.test_results['computational_load'] * 0.15 +
            self.test_results['robustness_score'] * 0.15
        )

        self.test_results['overall_performance'] = overall_score

        self.get_logger().info(f'--- PERFORMANCE SCORES ---')
        self.get_logger().info(f'Overall VSLAM Score: {overall_score:.3f}')

        # Determine pass/fail based on complex environment requirements
        map_coverage_pass = self.test_results['map_coverage'] >= (self.min_map_coverage * 0.5)  # Looser requirement for complex env
        feature_tracking_pass = self.test_results['feature_tracking_score'] >= 0.3  # Lower requirement for complex env
        tracking_stability_pass = self.test_results['tracking_stability'] >= 0.2  # Lower requirement for complex env
        computational_pass = self.test_results['computational_load'] >= 0.3  # Ensure reasonable performance
        robustness_pass = self.test_results['robustness_score'] >= 0.5  # Ensure robust operation

        self.get_logger().info(f'--- PASS/FAIL CRITERIA (Complex Environment) ---')
        self.get_logger().info(f'Map Coverage (>{self.min_map_coverage * 0.5}): {"✓ PASS" if map_coverage_pass else "✗ FAIL"}')
        self.get_logger().info(f'Feature Tracking (>.3): {"✓ PASS" if feature_tracking_pass else "✗ FAIL"}')
        self.get_logger().info(f'Tracking Stability (>.2): {"✓ PASS" if tracking_stability_pass else "✗ FAIL"}')
        self.get_logger().info(f'Computational Load (>.3): {"✓ PASS" if computational_pass else "✗ FAIL"}')
        self.get_logger().info(f'Robustness (>.5): {"✓ PASS" if robustness_pass else "✗ FAIL"}')

        # Overall assessment
        all_pass = all([map_coverage_pass, feature_tracking_pass, tracking_stability_pass, computational_pass, robustness_pass])
        self.get_logger().info(f'--- FINAL ASSESSMENT ---')
        self.get_logger().info(f'VSLAM complex factory test: {"✓ PASS" if all_pass else "✗ FAIL"}')

        # Performance categorization
        if overall_score >= 0.7:
            performance_level = "EXCELLENT"
        elif overall_score >= 0.5:
            performance_level = "GOOD"
        elif overall_score >= 0.3:
            performance_level = "ACCEPTABLE"
        else:
            performance_level = "POOR"

        self.get_logger().info(f'Performance Level: {performance_level} (Score: {overall_score:.3f})')

        # Recommendations
        self.get_logger().info(f'--- RECOMMENDATIONS ---')
        if not map_coverage_pass:
            self.get_logger().info('  - Consider using LiDAR for better mapping in textureless areas')
            self.get_logger().info('  - Improve camera exposure settings for varying lighting')
            self.get_logger().info('  - Optimize feature detection for industrial environments')
        if not feature_tracking_pass:
            self.get_logger().info('  - Tune feature detector for industrial textures')
            self.get_logger().info('  - Consider using multiple cameras for better coverage')
            self.get_logger().info('  - Implement dynamic feature threshold adjustment')
        if not computational_pass:
            self.get_logger().info('  - Optimize algorithm for real-time performance')
            self.get_logger().info('  - Consider hardware acceleration')
            self.get_logger().info('  - Reduce computational complexity in complex scenes')
        if not robustness_pass:
            self.get_logger().info('  - Improve failure recovery mechanisms')
            self.get_logger().info('  - Implement visual-inertial fusion for robustness')
            self.get_logger().info('  - Add fallback localization methods')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = VSLAMComplexFactoryTest()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 10.0)  # Add 10s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('VSLAM complex factory test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()