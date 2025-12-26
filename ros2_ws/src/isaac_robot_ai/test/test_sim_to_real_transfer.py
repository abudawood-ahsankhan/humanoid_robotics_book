#!/usr/bin/env python3
"""
Test script for verifying sim-to-real transfer capabilities with domain randomization.

This script evaluates how well policies trained in simulation with domain randomization
perform in real-world scenarios or in less randomized simulation conditions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float32
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque
import json
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean
import statistics


class SimToRealTransferTest(Node):
    def __init__(self):
        super().__init__('sim_to_real_transfer_test')

        # Declare parameters
        self.declare_parameter('test_duration', 300.0)  # 5 minutes
        self.declare_parameter('transfer_success_threshold', 0.7)  # 70% success rate
        self.declare_parameter('min_performance_drop', 0.2)  # 20% max performance drop
        self.declare_parameter('evaluation_frequency', 10.0)  # Hz
        self.declare_parameter('domain_randomization_enabled', True)
        self.declare_parameter('randomization_severity', 0.5)  # 0.0 to 1.0
        self.declare_parameter('real_world_validation', False)  # Whether to validate on real robot

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.transfer_success_threshold = self.get_parameter('transfer_success_threshold').value
        self.min_performance_drop = self.get_parameter('min_performance_drop').value
        self.evaluation_frequency = self.get_parameter('evaluation_frequency').value
        self.domain_randomization_enabled = self.get_parameter('domain_randomization_enabled').value
        self.randomization_severity = self.get_parameter('randomization_severity').value
        self.real_world_validation = self.get_parameter('real_world_validation').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize test variables
        self.test_start_time = None
        self.test_active = False
        self.evaluation_results = {
            'success_rate': 0.0,
            'average_completion_time': 0.0,
            'path_efficiency': 0.0,
            'collision_rate': 0.0,
            'navigation_stability': 0.0,
            'sensor_noise_robustness': 0.0,
            'lighting_variation_robustness': 0.0,
            'texture_variation_robustness': 0.0,
            'physics_variation_robustness': 0.0,
            'sim_to_real_gap': 0.0,
            'overall_transfer_score': 0.0
        }

        # Data collection
        self.navigation_successes = 0
        self.navigation_attempts = 0
        self.completion_times = deque(maxlen=100)
        self.path_efficiencies = deque(maxlen=100)
        self.collision_counts = deque(maxlen=100)
        self.navigation_trajectories = []
        self.current_trajectory = []
        self.test_episodes = []
        self.performance_metrics = {}
        self.data_lock = threading.Lock()

        # Navigation state
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.goal_position = np.array([5.0, 5.0, 0.0])
        self.navigation_active = False
        self.navigation_start_time = None
        self.navigation_end_time = None
        self.collision_detected = False

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/isaac_sim/joint_states',
            self.joint_state_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.collision_sub = self.create_subscription(
            Bool,
            '/collision_detected',
            self.collision_callback,
            sensor_qos
        )

        # Create publishers
        self.test_status_pub = self.create_publisher(
            String,
            '/transfer_test/status',
            sensor_qos
        )

        self.test_results_pub = self.create_publisher(
            String,
            '/transfer_test/results',
            sensor_qos
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            sensor_qos
        )

        # Create timer for evaluation
        self.evaluation_timer = self.create_timer(1.0 / self.evaluation_frequency, self.evaluate_performance)

        self.get_logger().info('Sim-to-Real Transfer Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Success threshold: {self.transfer_success_threshold}')
        self.get_logger().info(f'Domain randomization enabled: {self.domain_randomization_enabled}')
        self.get_logger().info(f'Randomization severity: {self.randomization_severity}')

    def odom_callback(self, msg):
        """Callback for odometry messages"""
        if not self.test_active:
            return

        with self.data_lock:
            # Extract position and orientation
            self.current_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])

            self.current_orientation = np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])

            self.current_velocity = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])

            # Add to current trajectory
            self.current_trajectory.append(self.current_position.copy())

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        if not self.test_active:
            return

        # Could be used to monitor joint behavior during transfer
        pass

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        if not self.test_active:
            return

        # Could be used to monitor balance during transfer
        pass

    def collision_callback(self, msg):
        """Callback for collision detection"""
        if not self.test_active:
            return

        with self.data_lock:
            if msg.data:
                self.collision_detected = True

    def start_test(self):
        """Start the sim-to-real transfer test"""
        self.get_logger().info('Starting sim-to-real transfer test...')
        self.get_logger().info('Testing policy transfer from randomized simulation to baseline conditions')

        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Initialize domain randomization
        if self.domain_randomization_enabled:
            self.initialize_domain_randomization(self.randomization_severity)

        # Run test episodes
        episode_count = 0
        while self.test_active and (self.get_clock().now().nanoseconds - self.test_start_time.nanoseconds) / 1e9 < self.test_duration:
            self.get_logger().info(f'Starting test episode {episode_count + 1}')

            # Set new goal
            self.set_new_goal()

            # Start navigation
            self.start_navigation()

            # Wait for navigation to complete or timeout
            start_time = self.get_clock().now()
            while self.navigation_active:
                if (self.get_clock().now().nanoseconds - start_time.nanoseconds) / 1e9 > 60.0:  # 60 sec timeout
                    self.get_logger().warn('Navigation timeout exceeded')
                    break
                time.sleep(0.1)

            # Evaluate episode
            self.evaluate_episode()

            # Increment episode counter
            episode_count += 1

            # Brief pause between episodes
            time.sleep(2.0)

        # End test
        self.test_active = False
        self.get_logger().info('Sim-to-real transfer test completed')

        # Process results
        self.process_test_results()

        # Generate report
        self.generate_test_report()

    def set_new_goal(self):
        """Set a new navigation goal"""
        with self.data_lock:
            # Generate random goal positions in a reasonable range
            self.goal_position = np.array([
                np.random.uniform(-5.0, 5.0),
                np.random.uniform(-5.0, 5.0),
                0.0
            ])

            # Create and publish goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = float(self.goal_position[0])
            goal_msg.pose.position.y = float(self.goal_position[1])
            goal_msg.pose.position.z = float(self.goal_position[2])
            goal_msg.pose.orientation.w = 1.0  # Default orientation

            self.goal_pub.publish(goal_msg)

    def start_navigation(self):
        """Start navigation to current goal"""
        with self.data_lock:
            self.navigation_active = True
            self.navigation_start_time = self.get_clock().now()
            self.current_trajectory = []
            self.collision_detected = False

    def evaluate_episode(self):
        """Evaluate the current navigation episode"""
        with self.data_lock:
            if self.navigation_end_time and self.navigation_start_time:
                # Calculate completion time
                completion_time = (self.navigation_end_time.nanoseconds - self.navigation_start_time.nanoseconds) / 1e9
                self.completion_times.append(completion_time)

                # Calculate path efficiency (ratio of straight-line distance to actual path length)
                if len(self.current_trajectory) > 1:
                    # Straight-line distance
                    straight_line_distance = np.linalg.norm(self.goal_position[:2] - self.current_trajectory[0][:2])

                    # Actual path length
                    path_segments = np.diff(self.current_trajectory, axis=0)
                    actual_path_length = np.sum(np.linalg.norm(path_segments, axis=1))

                    if straight_line_distance > 0:
                        path_efficiency = straight_line_distance / actual_path_length if actual_path_length > 0 else 1.0
                        self.path_efficiencies.append(min(1.0, path_efficiency))  # Cap at 1.0

                # Record collision count for this episode
                collision_count = 1 if self.collision_detected else 0
                self.collision_counts.append(collision_count)

                # Record episode results
                episode_result = {
                    'success': self.navigation_successful,
                    'completion_time': completion_time,
                    'path_efficiency': path_efficiency if 'path_efficiency' in locals() else 0.0,
                    'collisions': collision_count,
                    'trajectory': self.current_trajectory.copy(),
                    'start_position': self.current_trajectory[0] if self.current_trajectory else None,
                    'end_position': self.current_trajectory[-1] if self.current_trajectory else None,
                    'goal_position': self.goal_position.copy()
                }
                self.test_episodes.append(episode_result)

                if self.navigation_successful:
                    self.navigation_successes += 1

                self.navigation_attempts += 1

    def process_test_results(self):
        """Process collected data to calculate transfer metrics"""
        with self.data_lock:
            # Calculate success rate
            if self.navigation_attempts > 0:
                self.evaluation_results['success_rate'] = self.navigation_successes / self.navigation_attempts
            else:
                self.evaluation_results['success_rate'] = 0.0

            # Calculate average completion time
            if self.completion_times:
                self.evaluation_results['average_completion_time'] = float(np.mean(self.completion_times))
            else:
                self.evaluation_results['average_completion_time'] = 0.0

            # Calculate path efficiency
            if self.path_efficiencies:
                self.evaluation_results['path_efficiency'] = float(np.mean(self.path_efficiencies))
            else:
                self.evaluation_results['path_efficiency'] = 0.0

            # Calculate collision rate
            if self.collision_counts:
                self.evaluation_results['collision_rate'] = float(np.mean(self.collision_counts))
            else:
                self.evaluation_results['collision_rate'] = 0.0

            # Calculate navigation stability (based on trajectory smoothness)
            if self.navigation_trajectories:
                stability_scores = []
                for traj in self.navigation_trajectories:
                    if len(traj) > 2:
                        # Calculate average change in direction
                        directions = np.diff(traj, axis=0)
                        direction_changes = np.diff(directions, axis=0)
                        avg_change = np.mean(np.linalg.norm(direction_changes, axis=1))
                        # Lower change means more stable navigation
                        stability_scores.append(max(0.0, 1.0 - avg_change))

                if stability_scores:
                    self.evaluation_results['navigation_stability'] = float(np.mean(stability_scores))
                else:
                    self.evaluation_results['navigation_stability'] = 0.0
            else:
                self.evaluation_results['navigation_stability'] = 0.0

            # Calculate overall transfer score
            # Weight factors for different metrics
            success_weight = 0.3
            efficiency_weight = 0.2
            stability_weight = 0.2
            collision_weight = 0.2
            time_weight = 0.1

            self.evaluation_results['overall_transfer_score'] = (
                self.evaluation_results['success_rate'] * success_weight +
                self.evaluation_results['path_efficiency'] * efficiency_weight +
                self.evaluation_results['navigation_stability'] * stability_weight +
                max(0.0, (1.0 - self.evaluation_results['collision_rate'])) * collision_weight +
                max(0.0, 1.0 / (1.0 + self.evaluation_results['average_completion_time'] / 30.0)) * time_weight  # Normalize time (assume 30s is baseline)
            )

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== SIM-TO-REAL TRANSFER TEST REPORT ===')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')
        self.get_logger().info(f'Episodes Completed: {len(self.test_episodes)}')

        # Print individual metrics
        self.get_logger().info(f'--- TRANSFER METRICS ---')
        self.get_logger().info(f'Success Rate: {self.evaluation_results["success_rate"]:.3f} (Threshold: {self.transfer_success_threshold})')
        self.get_logger().info(f'Average Completion Time: {self.evaluation_results["average_completion_time"]:.3f}s')
        self.get_logger().info(f'Path Efficiency: {self.evaluation_results["path_efficiency"]:.3f}')
        self.get_logger().info(f'Collision Rate: {self.evaluation_results["collision_rate"]:.3f}')
        self.get_logger().info(f'Navigation Stability: {self.evaluation_results["navigation_stability"]:.3f}')
        self.get_logger().info(f'Overall Transfer Score: {self.evaluation_results["overall_transfer_score"]:.3f}')

        # Performance categorization
        if self.evaluation_results['overall_transfer_score'] >= 0.8:
            performance_level = "EXCELLENT"
        elif self.evaluation_results['overall_transfer_score'] >= 0.6:
            performance_level = "GOOD"
        elif self.evaluation_results['overall_transfer_score'] >= 0.4:
            performance_level = "ACCEPTABLE"
        else:
            performance_level = "POOR"

        self.get_logger().info(f'--- PERFORMANCE LEVEL ---')
        self.get_logger().info(f'Transfer Performance: {performance_level} (Score: {self.evaluation_results["overall_transfer_score"]:.3f})')

        # Pass/fail criteria
        success_pass = self.evaluation_results['success_rate'] >= self.transfer_success_threshold
        stability_pass = self.evaluation_results['navigation_stability'] >= 0.5
        efficiency_pass = self.evaluation_results['path_efficiency'] >= 0.3

        self.get_logger().info(f'--- PASS/FAIL CRITERIA ---')
        self.get_logger().info(f'Success Rate (>{self.transfer_success_threshold}): {"✓ PASS" if success_pass else "✗ FAIL"}')
        self.get_logger().info(f'Navigation Stability (>.5): {"✓ PASS" if stability_pass else "✗ FAIL"}')
        self.get_logger().info(f'Path Efficiency (>.3): {"✓ PASS" if efficiency_pass else "✗ FAIL"}')

        # Overall assessment
        all_pass = all([success_pass, stability_pass, efficiency_pass])
        self.get_logger().info(f'--- FINAL ASSESSMENT ---')
        self.get_logger().info(f'Sim-to-real transfer capability: {"✓ PASS" if all_pass else "✗ FAIL"}')

        # Recommendations
        self.get_logger().info(f'--- RECOMMENDATIONS ---')
        if not success_pass:
            self.get_logger().info('  - Increase domain randomization severity')
            self.get_logger().info('  - Add more diverse training scenarios')
            self.get_logger().info('  - Implement curriculum learning for gradual complexity increase')
        if not stability_pass:
            self.get_logger().info('  - Improve bipedal locomotion control')
            self.get_logger().info('  - Add balance feedback mechanisms')
            self.get_logger().info('  - Consider model predictive control for smoother trajectories')
        if not efficiency_pass:
            self.get_logger().info('  - Optimize path planning for efficiency')
            self.get_logger().info('  - Improve obstacle avoidance strategies')
            self.get_logger().info('  - Consider higher-level path optimization')

        # Publish results
        results_msg = String()
        results_msg.data = json.dumps({
            'timestamp': time.time(),
            'test_duration': self.test_duration,
            'episodes_completed': len(self.test_episodes),
            'metrics': self.evaluation_results,
            'assessment': {
                'pass': all_pass,
                'performance_level': performance_level,
                'recommendations': []
            }
        })
        self.test_results_pub.publish(results_msg)

    def initialize_domain_randomization(self, severity):
        """Initialize domain randomization with specified severity"""
        self.get_logger().info(f'Initializing domain randomization with severity: {severity}')

        # In a real implementation, this would interface with the domain randomization node
        # to set the randomization parameters to the specified severity level
        # For this test, we'll just log that we're using the specified severity

        # Publish randomization parameters
        randomization_msg = String()
        randomization_msg.data = json.dumps({
            'severity': severity,
            'components': {
                'lighting': severity,
                'textures': severity,
                'physics': severity,
                'sensors': severity
            }
        })

        # This would normally be published to the domain randomization node
        # self.randomization_pub.publish(randomization_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = SimToRealTransferTest()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 60.0)  # Add 60s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('Sim-to-Real Transfer Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()