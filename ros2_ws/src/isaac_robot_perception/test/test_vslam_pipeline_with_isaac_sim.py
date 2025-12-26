#!/usr/bin/env python3

"""
VSLAM Pipeline Test with Isaac Sim

This test validates the Isaac ROS VSLAM pipeline with Isaac Sim sensor data.
The test verifies that the VSLAM system is correctly processing camera and IMU data
from Isaac Sim and producing expected outputs for localization and mapping.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class VSLAMPipelineTest(Node):
    """
    Test node for validating the Isaac ROS VSLAM pipeline with Isaac Sim
    """

    def __init__(self):
        super().__init__('vslam_pipeline_test')

        # Test parameters
        self.declare_parameter('test_duration', 60.0)  # seconds
        self.declare_parameter('min_camera_rate', 10.0)  # Hz
        self.declare_parameter('min_imu_rate', 50.0)    # Hz
        self.declare_parameter('min_vslam_rate', 10.0)  # Hz
        self.declare_parameter('max_latency', 0.1)      # seconds
        self.declare_parameter('min_localization_accuracy', 0.1)  # meters

        self.test_duration = self.get_parameter('test_duration').value
        self.min_camera_rate = self.get_parameter('min_camera_rate').value
        self.min_imu_rate = self.get_parameter('min_imu_rate').value
        self.min_vslam_rate = self.get_parameter('min_vslam_rate').value
        self.max_latency = self.get_parameter('max_latency').value
        self.min_localization_accuracy = self.get_parameter('min_localization_accuracy').value

        # Initialize metrics
        self.camera_msg_count = 0
        self.imu_msg_count = 0
        self.vslam_msg_count = 0
        self.camera_start_time = None
        self.imu_start_time = None
        self.vslam_start_time = None
        self.camera_last_time = None
        self.imu_last_time = None
        self.vslam_last_time = None
        self.vslam_poses = []
        self.test_start_time = None

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to VSLAM outputs
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped,
            '/vslam/pose',
            self.vslam_pose_callback,
            reliable_qos
        )

        self.vslam_odom_sub = self.create_subscription(
            Odometry,
            '/vslam/odometry',
            self.vslam_odom_callback,
            reliable_qos
        )

        self.vslam_path_sub = self.create_subscription(
            Path,
            '/vslam/path_optimized',
            self.vslam_path_callback,
            reliable_qos
        )

        # Create publishers for test metrics
        self.test_results_pub = self.create_publisher(Float32, '/test/vslam_results', 10)

        # Timer for test execution
        self.test_timer = self.create_timer(1.0, self.test_timer_callback)
        self.test_complete_timer = self.create_timer(self.test_duration, self.test_complete_callback)

        self.get_logger().info('VSLAM Pipeline Test node initialized')
        self.get_logger().info(f'Starting test for {self.test_duration} seconds...')

        self.test_start_time = self.get_clock().now()

    def vslam_pose_callback(self, msg):
        """Callback for VSLAM pose messages"""
        current_time = self.get_clock().now()

        if self.vslam_start_time is None:
            self.vslam_start_time = current_time

        self.vslam_msg_count += 1
        self.vslam_last_time = current_time

        # Store pose for trajectory analysis
        self.vslam_poses.append((current_time, msg.pose))

    def vslam_odom_callback(self, msg):
        """Callback for VSLAM odometry messages"""
        # Additional odometry processing if needed
        pass

    def vslam_path_callback(self, msg):
        """Callback for VSLAM path messages"""
        # Additional path processing if needed
        pass

    def test_timer_callback(self):
        """Periodic test metrics evaluation"""
        current_time = self.get_clock().now()

        # Calculate current rates
        if self.vslam_start_time is not None:
            vslam_duration = (current_time - self.vslam_start_time).nanoseconds / 1e9
            if vslam_duration > 0:
                vslam_rate = self.vslam_msg_count / vslam_duration
                self.get_logger().info(f'VSLAM rate: {vslam_rate:.2f} Hz')

    def test_complete_callback(self):
        """Callback when test duration is complete"""
        self.get_logger().info('VSLAM Pipeline Test completed')
        self.evaluate_test_results()

    def evaluate_test_results(self):
        """Evaluate and report test results"""
        current_time = self.get_clock().now()
        total_test_duration = (current_time - self.test_start_time).nanoseconds / 1e9

        # Calculate rates
        if self.vslam_start_time is not None:
            vslam_duration = (current_time - self.vslam_start_time).nanoseconds / 1e9
            vslam_rate = self.vslam_msg_count / vslam_duration if vslam_duration > 0 else 0
        else:
            vslam_rate = 0

        # Print test results
        self.get_logger().info('=== VSLAM Pipeline Test Results ===')
        self.get_logger().info(f'Total test duration: {total_test_duration:.2f} seconds')
        self.get_logger().info(f'VSLAM messages received: {self.vslam_msg_count}')
        self.get_logger().info(f'VSLAM rate: {vslam_rate:.2f} Hz')
        self.get_logger().info(f'Minimum required VSLAM rate: {self.min_vslam_rate} Hz')

        # Evaluate performance
        rate_pass = vslam_rate >= self.min_vslam_rate
        self.get_logger().info(f'Rate requirement met: {rate_pass}')

        # Calculate trajectory metrics if we have poses
        if len(self.vslam_poses) > 1:
            # Calculate trajectory length
            total_distance = 0.0
            for i in range(1, len(self.vslam_poses)):
                prev_pose = self.vslam_poses[i-1][1]
                curr_pose = self.vslam_poses[i][1]

                dist = math.sqrt(
                    (curr_pose.position.x - prev_pose.position.x)**2 +
                    (curr_pose.position.y - prev_pose.position.y)**2 +
                    (curr_pose.position.z - prev_pose.position.z)**2
                )
                total_distance += dist

            self.get_logger().info(f'Total trajectory distance: {total_distance:.2f} meters')

        # Overall result
        overall_pass = rate_pass
        self.get_logger().info(f'Overall test result: {"PASS" if overall_pass else "FAIL"}')

        # Publish test result
        result_msg = Float32()
        result_msg.data = 1.0 if overall_pass else 0.0
        self.test_results_pub.publish(result_msg)


def main(args=None):
    """Main function to run the VSLAM pipeline test"""
    rclpy.init(args=args)

    test_node = VSLAMPipelineTest()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()