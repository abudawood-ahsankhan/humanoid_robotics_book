#!/usr/bin/env python3
"""
Isaac Sim Bridge Integration Test

This script tests the integration between Isaac Sim and the ROS 2 navigation system,
verifying that sensor data flows correctly and navigation commands are properly executed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool, Header
from builtin_interfaces.msg import Time
import time
import threading
from collections import deque
import numpy as np
import subprocess
import signal
import os


class IsaacSimBridgeIntegrationTest(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge_integration_test')

        # Declare parameters
        self.declare_parameter('test_duration', 120.0)  # 2 minutes
        self.declare_parameter('robot_spawn_timeout', 30.0)  # seconds
        self.declare_parameter('sensor_timeout', 10.0)  # seconds
        self.declare_parameter('navigation_timeout', 60.0)  # seconds
        self.declare_parameter('expected_topics', [
            '/sensors/camera/image_raw',
            '/sensors/lidar/points',
            '/sensors/imu/data',
            '/isaac_sim/joint_states',
            '/odom',
            '/tf'
        ])
        self.declare_parameter('navigation_goal_x', 5.0)
        self.declare_parameter('navigation_goal_y', 5.0)
        self.declare_parameter('navigation_goal_tolerance', 0.5)

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.spawn_timeout = self.get_parameter('robot_spawn_timeout').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.expected_topics = self.get_parameter('expected_topics').value
        self.navigation_goal_x = self.get_parameter('navigation_goal_x').value
        self.navigation_goal_y = self.get_parameter('navigation_goal_y').value
        self.navigation_tolerance = self.get_parameter('navigation_goal_tolerance').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize test variables
        self.test_start_time = None
        self.test_active = False
        self.bridge_connected = False
        self.robot_spawned = False
        self.all_sensors_active = False
        self.navigation_working = False
        self.navigation_success = False

        # Data collection
        self.received_topics = set()
        self.camera_messages = deque(maxlen=100)
        self.lidar_messages = deque(maxlen=100)
        self.imu_messages = deque(maxlen=100)
        self.joint_state_messages = deque(maxlen=100)
        self.odom_messages = deque(maxlen=100)
        self.tf_messages = deque(maxlen=100)
        self.navigation_status = "IDLE"
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.goal_position = np.array([self.navigation_goal_x, self.navigation_goal_y, 0.0])
        self.navigation_start_time = None
        self.navigation_end_time = None
        self.data_lock = threading.Lock()

        # Create subscribers for sensor data
        self.camera_sub = self.create_subscription(
            Image,
            '/sensors/camera/image_raw',
            self.camera_callback,
            sensor_qos
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/sensors/lidar/points',
            self.lidar_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/isaac_sim/joint_states',
            self.joint_state_callback,
            sensor_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        # Create publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            sensor_qos
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            sensor_qos
        )

        self.test_status_pub = self.create_publisher(
            String,
            '/integration_test/status',
            sensor_qos
        )

        self.get_logger().info('Isaac Sim Bridge Integration Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Navigation goal: ({self.navigation_goal_x}, {self.navigation_goal_y})')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        with self.data_lock:
            self.received_topics.add('/sensors/camera/image_raw')
            self.camera_messages.append(msg)
            self.get_logger().debug(f'Received camera message with shape {msg.width}x{msg.height}')

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        with self.data_lock:
            self.received_topics.add('/sensors/lidar/points')
            self.lidar_messages.append(msg)
            self.get_logger().debug(f'Received LiDAR message with {msg.width * msg.height} points')

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        with self.data_lock:
            self.received_topics.add('/sensors/imu/data')
            self.imu_messages.append(msg)
            self.get_logger().debug('Received IMU message')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        with self.data_lock:
            self.received_topics.add('/isaac_sim/joint_states')
            self.joint_state_messages.append(msg)
            self.get_logger().debug(f'Received joint state with {len(msg.name)} joints')

    def odom_callback(self, msg):
        """Callback for odometry messages"""
        with self.data_lock:
            self.received_topics.add('/odom')
            self.odom_messages.append(msg)

            # Update current position
            self.current_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])

            self.get_logger().debug(f'Received odometry message, position: {self.current_position}')

    def run_integration_test(self):
        """Run the complete integration test"""
        self.get_logger().info('Starting Isaac Sim Bridge Integration Test...')

        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Phase 1: Verify Isaac Sim connection
        self.get_logger().info('Phase 1: Verifying Isaac Sim connection and robot spawning...')
        if not self.verify_isaac_sim_connection():
            self.get_logger().error('Isaac Sim connection verification failed')
            return False

        # Phase 2: Verify sensor data flow
        self.get_logger().info('Phase 2: Verifying sensor data flow...')
        if not self.verify_sensor_data_flow():
            self.get_logger().error('Sensor data flow verification failed')
            return False

        # Phase 3: Test navigation system
        self.get_logger().info('Phase 3: Testing navigation system integration...')
        if not self.test_navigation_system():
            self.get_logger().error('Navigation system test failed')
            return False

        # Phase 4: Performance evaluation
        self.get_logger().info('Phase 4: Evaluating system performance...')
        self.evaluate_performance()

        # Phase 5: Generate test report
        self.get_logger().info('Phase 5: Generating test report...')
        self.generate_test_report()

        self.get_logger().info('Isaac Sim Bridge Integration Test completed successfully!')
        return True

    def verify_isaac_sim_connection(self):
        """Verify that Isaac Sim is connected and robot is spawned"""
        start_time = self.get_clock().now()

        while (self.get_clock().now().nanoseconds - start_time.nanoseconds) / 1e9 < self.spawn_timeout:
            # Check if we can reach Isaac Sim services/topics
            available_topics = [name for name, _ in self.get_topic_names_and_types()]

            # Check for essential Isaac Sim topics
            isaac_sim_topics = [topic for topic in available_topics if 'isaac_sim' in topic]

            if len(isaac_sim_topics) > 0:
                self.get_logger().info(f'Found {len(isaac_sim_topics)} Isaac Sim topics: {isaac_sim_topics}')

                # Check if robot is spawned by looking for joint states
                if '/isaac_sim/joint_states' in available_topics:
                    self.get_logger().info('Robot joint states topic detected - robot likely spawned')

                    # Try to get robot joint information
                    try:
                        # Send a small velocity command to verify robot responsiveness
                        cmd_msg = Twist()
                        cmd_msg.linear.x = 0.1
                        cmd_msg.angular.z = 0.05
                        self.cmd_vel_pub.publish(cmd_msg)

                        time.sleep(1.0)  # Allow time for response

                        # Check if odometry is being published (indicating robot movement)
                        initial_odom_count = len(self.odom_messages)
                        time.sleep(2.0)
                        final_odom_count = len(self.odom_messages)

                        if final_odom_count > initial_odom_count:
                            self.get_logger().info('Robot movement detected - bridge connection verified')
                            return True
                        else:
                            self.get_logger().warn('No robot movement detected after command')
                    except Exception as e:
                        self.get_logger().error(f'Error testing robot responsiveness: {e}')

            self.get_logger().info('Waiting for Isaac Sim connection...')
            time.sleep(1.0)

        self.get_logger().error(f'Isaac Sim connection not established within {self.spawn_timeout}s timeout')
        return False

    def verify_sensor_data_flow(self):
        """Verify that all sensor data is flowing correctly"""
        start_time = self.get_clock().now()

        while (self.get_clock().now().nanoseconds - start_time.nanoseconds) / 1e9 < self.sensor_timeout:
            with self.data_lock:
                # Check which topics have received data
                active_topics = set()
                if len(self.camera_messages) > 0:
                    active_topics.add('/sensors/camera/image_raw')
                if len(self.lidar_messages) > 0:
                    active_topics.add('/sensors/lidar/points')
                if len(self.imu_messages) > 0:
                    active_topics.add('/sensors/imu/data')
                if len(self.joint_state_messages) > 0:
                    active_topics.add('/isaac_sim/joint_states')
                if len(self.odom_messages) > 0:
                    active_topics.add('/odom')

            self.get_logger().info(f'Active sensor topics: {list(active_topics)}')

            # Check if all expected sensor topics are active
            missing_topics = set(self.expected_topics) - active_topics - {'/tf'}  # TF may not always be active
            if len(missing_topics) == 0:
                self.get_logger().info('All sensor data streams are active')
                return True

            self.get_logger().info(f'Missing sensor topics: {missing_topics}')
            time.sleep(1.0)

        self.get_logger().error(f'Not all sensor topics active within {self.sensor_timeout}s timeout')
        return False

    def test_navigation_system(self):
        """Test the navigation system integration"""
        self.get_logger().info('Testing navigation system...')

        # Create navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(self.navigation_goal_x)
        goal_msg.pose.position.y = float(self.navigation_goal_y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # Default orientation

        # Record start position
        with self.data_lock:
            start_position = self.current_position.copy()

        # Publish navigation goal
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Published navigation goal to ({self.navigation_goal_x}, {self.navigation_goal_y})')

        # Wait for navigation to complete or timeout
        start_time = self.get_clock().now()
        reached_goal = False

        while (self.get_clock().now().nanoseconds - start_time.nanoseconds) / 1e9 < self.navigation_timeout:
            with self.data_lock:
                # Check if we've reached the goal
                distance_to_goal = np.linalg.norm(self.current_position[:2] - self.goal_position[:2])

                if distance_to_goal <= self.navigation_tolerance:
                    reached_goal = True
                    self.navigation_end_time = self.get_clock().now()
                    self.get_logger().info(f'Navigation goal reached! Distance: {distance_to_goal:.3f}m')
                    break

            time.sleep(0.1)

        if reached_goal:
            self.get_logger().info('Navigation test completed successfully')
            return True
        else:
            self.get_logger().error('Navigation test failed - goal not reached within timeout')
            return False

    def evaluate_performance(self):
        """Evaluate system performance metrics"""
        self.get_logger().info('Evaluating system performance...')

        with self.data_lock:
            # Calculate sensor data rates
            if len(self.camera_messages) > 1:
                first_time = self.camera_messages[0].header.stamp.sec + self.camera_messages[0].header.stamp.nanosec / 1e9
                last_time = self.camera_messages[-1].header.stamp.sec + self.camera_messages[-1].header.stamp.nanosec / 1e9
                duration = last_time - first_time
                if duration > 0:
                    camera_rate = len(self.camera_messages) / duration
                    self.get_logger().info(f'Camera data rate: {camera_rate:.2f} Hz')

            if len(self.lidar_messages) > 1:
                first_time = self.lidar_messages[0].header.stamp.sec + self.lidar_messages[0].header.stamp.nanosec / 1e9
                last_time = self.lidar_messages[-1].header.stamp.sec + self.lidar_messages[-1].header.stamp.nanosec / 1e9
                duration = last_time - first_time
                if duration > 0:
                    lidar_rate = len(self.lidar_messages) / duration
                    self.get_logger().info(f'LiDAR data rate: {lidar_rate:.2f} Hz')

            if len(self.odom_messages) > 1:
                first_time = self.odom_messages[0].header.stamp.sec + self.odom_messages[0].header.stamp.nanosec / 1e9
                last_time = self.odom_messages[-1].header.stamp.sec + self.odom_messages[-1].header.stamp.nanosec / 1e9
                duration = last_time - first_time
                if duration > 0:
                    odom_rate = len(self.odom_messages) / duration
                    self.get_logger().info(f'Odometry data rate: {odom_rate:.2f} Hz')

            # Calculate navigation performance
            if self.navigation_end_time and self.navigation_start_time:
                nav_time = (self.navigation_end_time.nanoseconds - self.navigation_start_time.nanoseconds) / 1e9
                self.get_logger().info(f'Navigation completion time: {nav_time:.2f} seconds')

                # Calculate path efficiency
                straight_line_distance = np.linalg.norm(self.goal_position[:2] - self.start_position[:2])
                actual_path_length = self.calculate_path_length()
                if straight_line_distance > 0:
                    path_efficiency = straight_line_distance / actual_path_length if actual_path_length > 0 else 0
                    self.get_logger().info(f'Path efficiency: {path_efficiency:.3f}')

    def calculate_path_length(self):
        """Calculate the length of the robot's path"""
        with self.data_lock:
            if len(self.odom_messages) < 2:
                return 0.0

            positions = []
            for msg in self.odom_messages:
                pos = msg.pose.pose.position
                positions.append([pos.x, pos.y])

            if len(positions) < 2:
                return 0.0

            path_length = 0.0
            for i in range(1, len(positions)):
                segment_length = np.linalg.norm(np.array(positions[i]) - np.array(positions[i-1]))
                path_length += segment_length

            return path_length

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== ISAAC SIM BRIDGE INTEGRATION TEST REPORT ===')

        with self.data_lock:
            # Sensor data summary
            self.get_logger().info(f'--- SENSOR DATA SUMMARY ---')
            self.get_logger().info(f'Camera messages received: {len(self.camera_messages)}')
            self.get_logger().info(f'LiDAR messages received: {len(self.lidar_messages)}')
            self.get_logger().info(f'IMU messages received: {len(self.imu_messages)}')
            self.get_logger().info(f'Joint state messages received: {len(self.joint_state_messages)}')
            self.get_logger().info(f'Odometry messages received: {len(self.odom_messages)}')

            # Navigation results
            self.get_logger().info(f'--- NAVIGATION RESULTS ---')
            self.get_logger().info(f'Navigation goal: ({self.navigation_goal_x}, {self.navigation_goal_y})')
            self.get_logger().info(f'Start position: ({self.start_position[0]:.2f}, {self.start_position[1]:.2f})')
            self.get_logger().info(f'Final position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})')
            self.get_logger().info(f'Distance to goal: {np.linalg.norm(self.current_position[:2] - self.goal_position[:2]):.3f}m')
            self.get_logger().info(f'Navigation success: {"YES" if self.navigation_success else "NO"}')

            # Performance metrics
            self.get_logger().info(f'--- PERFORMANCE METRICS ---')
            self.get_logger().info(f'Test duration: {self.test_duration} seconds')
            self.get_logger().info(f'All sensors active: {self.all_sensors_active}')
            self.get_logger().info(f'Bridge connected: {self.bridge_connected}')
            self.get_logger().info(f'Robot spawned: {self.robot_spawned}')

            # Overall assessment
            all_checks_passed = all([
                len(self.camera_messages) > 0,
                len(self.lidar_messages) > 0,
                len(self.imu_messages) > 0,
                len(self.joint_state_messages) > 0,
                len(self.odom_messages) > 0,
                self.navigation_success
            ])

            self.get_logger().info(f'--- FINAL ASSESSMENT ---')
            self.get_logger().info(f'Integration test: {"PASS" if all_checks_passed else "FAIL"}')

            if all_checks_passed:
                self.get_logger().info('✓ Isaac Sim-ROS 2 bridge integration is functioning correctly')
                self.get_logger().info('✓ All sensor data streams are active')
                self.get_logger().info('✓ Navigation system responds to goals')
                self.get_logger().info('✓ Robot successfully navigates to goal')
            else:
                self.get_logger().info('✗ Isaac Sim-ROS 2 bridge integration has issues')
                if len(self.camera_messages) == 0:
                    self.get_logger().info('  - Camera data not received')
                if len(self.lidar_messages) == 0:
                    self.get_logger().info('  - LiDAR data not received')
                if len(self.imu_messages) == 0:
                    self.get_logger().info('  - IMU data not received')
                if len(self.joint_state_messages) == 0:
                    self.get_logger().info('  - Joint state data not received')
                if len(self.odom_messages) == 0:
                    self.get_logger().info('  - Odometry data not received')
                if not self.navigation_success:
                    self.get_logger().info('  - Navigation system not working properly')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = IsaacSimBridgeIntegrationTest()

    try:
        # Run the integration test
        success = test_node.run_integration_test()

        if not success:
            test_node.get_logger().error('Integration test failed')
            exit(1)
        else:
            test_node.get_logger().info('Integration test completed successfully')
    except KeyboardInterrupt:
        test_node.get_logger().info('Integration test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()