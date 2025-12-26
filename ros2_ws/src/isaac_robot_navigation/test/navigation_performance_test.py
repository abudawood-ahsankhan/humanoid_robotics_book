#!/usr/bin/env python3
"""
Navigation Performance Test for Humanoid Robot in Navigation Environment

This script tests the navigation performance of the humanoid robot
in the humanoid_navigation environment, evaluating path planning,
obstacle avoidance, and bipedal locomotion capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Time
import numpy as np
import time
import threading
from collections import deque
import math
import random


class NavigationPerformanceTest(Node):
    def __init__(self):
        super().__init__('navigation_performance_test')

        # Declare parameters
        self.declare_parameter('test_duration', 300.0)  # 5 minutes
        self.declare_parameter('navigation_goal_topic', '/navigate_to_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('laser_scan_topic', '/scan')
        self.declare_parameter('max_execution_time', 60.0)  # seconds
        self.declare_parameter('min_success_rate', 0.8)  # 80% success rate
        self.declare_parameter('max_deviation_from_path', 0.3)  # meters
        self.declare_parameter('min_navigation_speed', 0.1)  # m/s
        self.declare_parameter('max_collision_count', 1)  # maximum collisions allowed
        self.declare_parameter('bipedal_stability_threshold', 0.7)  # stability score

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.navigation_goal_topic = self.get_parameter('navigation_goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.laser_scan_topic = self.get_parameter('laser_scan_topic').value
        self.max_execution_time = self.get_parameter('max_execution_time').value
        self.min_success_rate = self.get_parameter('min_success_rate').value
        self.max_deviation_from_path = self.get_parameter('max_deviation_from_path').value
        self.min_navigation_speed = self.get_parameter('min_navigation_speed').value
        self.max_collision_count = self.get_parameter('max_collision_count').value
        self.bipedal_stability_threshold = self.get_parameter('bipedal_stability_threshold').value

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
            'success_rate': 0.0,
            'average_execution_time': 0.0,
            'path_deviation_score': 0.0,
            'speed_score': 0.0,
            'collision_rate': 0.0,
            'bipedal_stability': 0.0,
            'navigation_efficiency': 0.0,
            'total_tests': 0,
            'successful_navigations': 0,
            'failed_navigations': 0,
            'collisions_detected': 0,
            'average_speed': 0.0,
            'path_efficiency': 0.0
        }

        # Data collection
        self.odom_data = deque(maxlen=1000)
        self.cmd_vel_data = deque(maxlen=1000)
        self.path_data = None
        self.laser_scan_data = None
        self.navigation_goals = []
        self.navigation_results = []
        self.test_trajectories = []
        self.collision_events = []
        self.bipedal_balance_scores = deque(maxlen=100)
        self.data_lock = threading.Lock()

        # Navigation state
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.navigation_active = False
        self.navigation_start_time = None
        self.navigation_end_time = None

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            sensor_qos
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            sensor_qos
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            sensor_qos
        )

        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            self.laser_scan_topic,
            self.laser_scan_callback,
            sensor_qos
        )

        # Create publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.navigation_goal_topic.replace('/navigate_to_pose', '/goal_pose'),
            sensor_qos
        )

        self.get_logger().info('Navigation Performance Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Max execution time: {self.max_execution_time} seconds')
        self.get_logger().info(f'Min success rate: {self.min_success_rate}')

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

            # Store odometry data
            odom_info = {
                'timestamp': msg.header.stamp,
                'position': self.current_position.copy(),
                'orientation': self.current_orientation.copy(),
                'velocity': self.current_velocity.copy()
            }
            self.odom_data.append(odom_info)

    def cmd_vel_callback(self, msg):
        """Callback for velocity command messages"""
        if not self.test_active:
            return

        with self.data_lock:
            cmd_info = {
                'timestamp': self.get_clock().now().to_msg(),
                'linear': np.array([msg.linear.x, msg.linear.y, msg.linear.z]),
                'angular': np.array([msg.angular.x, msg.angular.y, msg.angular.z])
            }
            self.cmd_vel_data.append(cmd_info)

    def path_callback(self, msg):
        """Callback for path messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.path_data = msg

    def laser_scan_callback(self, msg):
        """Callback for laser scan messages"""
        if not self.test_active:
            return

        with self.data_lock:
            # Check for potential collisions
            min_distance = min([d for d in msg.ranges if not np.isnan(d) and d > 0], default=float('inf'))

            if min_distance < 0.5:  # Potential collision threshold
                collision_event = {
                    'timestamp': msg.header.stamp,
                    'distance': min_distance,
                    'position': self.current_position.copy()
                }
                self.collision_events.append(collision_event)
                self.test_results['collisions_detected'] += 1

            self.laser_scan_data = msg

    def start_test(self):
        """Start the navigation performance test"""
        self.get_logger().info('Starting navigation performance test in humanoid_navigation environment...')
        self.get_logger().info('Testing path planning, obstacle avoidance, and bipedal locomotion...')

        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Generate test waypoints in the humanoid_navigation environment
        test_waypoints = self.generate_test_waypoints()
        self.get_logger().info(f'Generated {len(test_waypoints)} test waypoints')

        # Execute navigation tests
        for i, waypoint in enumerate(test_waypoints):
            if not self.test_active:
                break

            self.get_logger().info(f'Executing navigation test {i+1}/{len(test_waypoints)} to {waypoint}')

            success = self.execute_navigation_test(waypoint)
            self.navigation_results.append(success)

            if success:
                self.test_results['successful_navigations'] += 1
            else:
                self.test_results['failed_navigations'] += 1

            self.test_results['total_tests'] += 1

            # Brief pause between tests
            time.sleep(2.0)

        # Stop test
        self.test_active = False
        self.get_logger().info('Navigation performance test completed')

        # Process results
        self.process_test_results()

        # Generate report
        self.generate_test_report()

    def generate_test_waypoints(self):
        """Generate test waypoints in the humanoid_navigation environment"""
        # Define waypoints in the humanoid_navigation environment
        # These would correspond to interesting locations in the environment
        waypoints = [
            # Start area
            [1.0, 1.0, 0.0],
            [2.0, 2.0, 0.0],

            # Navigation challenge areas
            [5.0, 1.0, 0.0],
            [5.0, 5.0, 0.0],
            [1.0, 5.0, 0.0],

            # Obstacle course
            [3.0, 3.0, 0.0],
            [7.0, 2.0, 0.0],
            [7.0, 7.0, 0.0],
            [2.0, 7.0, 0.0],

            # Final destination
            [0.5, 0.5, 0.0]
        ]

        return waypoints

    def execute_navigation_test(self, target_waypoint):
        """Execute a single navigation test to the target waypoint"""
        with self.data_lock:
            start_position = self.current_position.copy()

        # Create navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(target_waypoint[0])
        goal_msg.pose.position.y = float(target_waypoint[1])
        goal_msg.pose.position.z = float(target_waypoint[2])
        goal_msg.pose.orientation.w = 1.0  # Default orientation

        # Publish goal
        self.goal_pub.publish(goal_msg)

        # Record start time
        test_start_time = self.get_clock().now()
        self.navigation_active = True
        self.navigation_start_time = test_start_time

        # Wait for navigation to complete or timeout
        success = False
        while self.navigation_active:
            current_time = self.get_clock().now()
            elapsed = (current_time.nanoseconds - test_start_time.nanoseconds) / 1e9

            if elapsed > self.max_execution_time:
                self.get_logger().warn(f'Navigation test timed out after {elapsed:.2f}s')
                break

            # Check if we reached the goal (within tolerance)
            with self.data_lock:
                distance_to_goal = np.linalg.norm(
                    np.array([target_waypoint[0], target_waypoint[1]]) -
                    self.current_position[:2]
                )

                if distance_to_goal < 0.5:  # Within 50cm of goal
                    self.navigation_end_time = current_time
                    success = True
                    self.get_logger().info(f'Navigation test succeeded! Distance to goal: {distance_to_goal:.3f}m')
                    break

            time.sleep(0.1)

        self.navigation_active = False

        # Calculate test metrics
        if success and self.navigation_end_time:
            execution_time = (self.navigation_end_time.nanoseconds - test_start_time.nanoseconds) / 1e9
            self.get_logger().info(f'Navigation completed in {execution_time:.2f}s')

        return success

    def process_test_results(self):
        """Process collected data to calculate test metrics"""
        with self.data_lock:
            # Calculate success rate
            if self.test_results['total_tests'] > 0:
                self.test_results['success_rate'] = self.test_results['successful_navigations'] / self.test_results['total_tests']
            else:
                self.test_results['success_rate'] = 0.0

            # Calculate collision rate
            self.test_results['collision_rate'] = self.test_results['collisions_detected'] / max(1, self.test_results['total_tests'])

            # Calculate average speed from odometry data
            if len(self.odom_data) > 1:
                velocities = [np.linalg.norm(od['velocity'][:2]) for od in self.odom_data]
                self.test_results['average_speed'] = np.mean(velocities)

                # Calculate speed score (higher is better, but bounded)
                self.test_results['speed_score'] = min(1.0, self.test_results['average_speed'] / self.min_navigation_speed)

            # Calculate path efficiency based on trajectory
            if len(self.odom_data) > 2:
                # Calculate actual path length
                positions = [od['position'][:2] for od in self.odom_data]
                if len(positions) > 1:
                    path_lengths = [np.linalg.norm(positions[i+1] - positions[i]) for i in range(len(positions)-1)]
                    actual_path_length = sum(path_lengths)

                    # Calculate straight-line distance (if we have start and end points)
                    if len(positions) >= 2:
                        straight_line_distance = np.linalg.norm(positions[-1] - positions[0])
                        if straight_line_distance > 0:
                            self.test_results['path_efficiency'] = straight_line_distance / actual_path_length if actual_path_length > 0 else 1.0

            # Calculate bipedal stability score (simulated)
            # In a real implementation, this would come from balance sensors
            if len(self.bipedal_balance_scores) > 0:
                avg_balance = np.mean(self.bipedal_balance_scores)
                self.test_results['bipedal_stability'] = avg_balance
            else:
                # Simulated stability based on smoothness of motion
                if len(self.cmd_vel_data) > 10:
                    # Calculate smoothness of velocity commands
                    vel_changes = []
                    for i in range(1, min(10, len(self.cmd_vel_data))):
                        prev_vel = np.linalg.norm(self.cmd_vel_data[-i-1]['linear'])
                        curr_vel = np.linalg.norm(self.cmd_vel_data[-i]['linear'])
                        vel_changes.append(abs(curr_vel - prev_vel))

                    if vel_changes:
                        avg_change = np.mean(vel_changes)
                        # Lower velocity changes indicate smoother, more stable motion
                        self.test_results['bipedal_stability'] = max(0.0, 1.0 - avg_change * 2.0)

            # Calculate overall navigation efficiency
            self.test_results['navigation_efficiency'] = (
                self.test_results['success_rate'] * 0.4 +
                self.test_results['speed_score'] * 0.2 +
                self.test_results['path_efficiency'] * 0.2 +
                self.test_results['bipedal_stability'] * 0.2
            )

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== NAVIGATION PERFORMANCE TEST REPORT ===')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')

        # Print individual metrics
        self.get_logger().info(f'--- NAVIGATION METRICS ---')
        self.get_logger().info(f'Total Tests: {self.test_results["total_tests"]}')
        self.get_logger().info(f'Successful Navigations: {self.test_results["successful_navigations"]}')
        self.get_logger().info(f'Failed Navigations: {self.test_results["failed_navigations"]}')
        self.get_logger().info(f'Success Rate: {self.test_results["success_rate"]:.3f} (Required: {self.min_success_rate})')
        self.get_logger().info(f'Average Speed: {self.test_results["average_speed"]:.3f} m/s (Required: >{self.min_navigation_speed})')
        self.get_logger().info(f'Collisions Detected: {self.test_results["collisions_detected"]} (Max allowed: {self.max_collision_count})')
        self.get_logger().info(f'Bipedal Stability: {self.test_results["bipedal_stability"]:.3f} (Required: >{self.bipedal_stability_threshold})')
        self.get_logger().info(f'Path Efficiency: {self.test_results["path_efficiency"]:.3f}')

        # Calculate overall score
        self.get_logger().info(f'--- PERFORMANCE SCORES ---')
        self.get_logger().info(f'Navigation Efficiency Score: {self.test_results["navigation_efficiency"]:.3f}')

        # Determine pass/fail criteria
        success_rate_pass = self.test_results['success_rate'] >= self.min_success_rate
        speed_pass = self.test_results['average_speed'] >= self.min_navigation_speed
        collision_pass = self.test_results['collisions_detected'] <= self.max_collision_count
        stability_pass = self.test_results['bipedal_stability'] >= self.bipedal_stability_threshold

        self.get_logger().info(f'--- PASS/FAIL CRITERIA ---')
        self.get_logger().info(f'Success Rate (>{self.min_success_rate}): {"✓ PASS" if success_rate_pass else "✗ FAIL"}')
        self.get_logger().info(f'Navigation Speed (>{self.min_navigation_speed} m/s): {"✓ PASS" if speed_pass else "✗ FAIL"}')
        self.get_logger().info(f'Collision Rate (≤{self.max_collision_count}): {"✓ PASS" if collision_pass else "✗ FAIL"}')
        self.get_logger().info(f'Bipedal Stability (>{self.bipedal_stability_threshold}): {"✓ PASS" if stability_pass else "✗ FAIL"}')

        # Overall assessment
        all_pass = all([success_rate_pass, speed_pass, collision_pass, stability_pass])
        self.get_logger().info(f'--- FINAL ASSESSMENT ---')
        self.get_logger().info(f'Navigation performance test: {"✓ PASS" if all_pass else "✗ FAIL"}')

        # Performance categorization
        if self.test_results['navigation_efficiency'] >= 0.8:
            performance_level = "EXCELLENT"
        elif self.test_results['navigation_efficiency'] >= 0.6:
            performance_level = "GOOD"
        elif self.test_results['navigation_efficiency'] >= 0.4:
            performance_level = "ACCEPTABLE"
        else:
            performance_level = "POOR"

        self.get_logger().info(f'Performance Level: {performance_level} (Score: {self.test_results["navigation_efficiency"]:.3f})')

        # Recommendations
        self.get_logger().info(f'--- RECOMMENDATIONS ---')
        if not success_rate_pass:
            self.get_logger().info('  - Improve path planning algorithm')
            self.get_logger().info('  - Enhance obstacle detection and avoidance')
            self.get_logger().info('  - Optimize local planner parameters')
        if not speed_pass:
            self.get_logger().info('  - Adjust velocity limits for better performance')
            self.get_logger().info('  - Optimize trajectory generation')
            self.get_logger().info('  - Consider terrain-specific speed profiles')
        if not collision_pass:
            self.get_logger().info('  - Improve obstacle detection sensitivity')
            self.get_logger().info('  - Increase safety margins in costmap')
            self.get_logger().info('  - Enhance local planner obstacle avoidance')
        if not stability_pass:
            self.get_logger().info('  - Optimize bipedal locomotion controller')
            self.get_logger().info('  - Improve balance feedback systems')
            self.get_logger().info('  - Adjust step timing for better stability')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = NavigationPerformanceTest()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 60.0)  # Add 60s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('Navigation performance test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()