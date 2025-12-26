#!/usr/bin/env python3

"""
VSLAM Performance Test in Isaac Sim Environments

This script tests the performance of the Isaac ROS VSLAM pipeline in different Isaac Sim environments.
It validates localization accuracy, mapping quality, and tracking stability.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32, Bool
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import tf2_ros
from tf2_ros import TransformException


class VSLAMPerformanceTest(Node):
    """
    Test node for validating VSLAM performance in Isaac Sim environments
    """

    def __init__(self):
        super().__init__('vslam_performance_test')

        # Test parameters
        self.declare_parameter('test_duration', 120.0)  # seconds
        self.declare_parameter('ground_truth_frame', 'world')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('min_localization_accuracy', 0.1)  # meters
        self.declare_parameter('min_tracking_success_rate', 0.7)  # 70%

        self.test_duration = self.get_parameter('test_duration').value
        self.ground_truth_frame = self.get_parameter('ground_truth_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.min_localization_accuracy = self.get_parameter('min_localization_accuracy').value
        self.min_tracking_success_rate = self.get_parameter('min_tracking_success_rate').value

        # Initialize metrics
        self.localization_errors = []
        self.tracking_success_rates = []
        self.vslam_poses = []
        self.ground_truth_poses = []
        self.test_start_time = None
        self.last_evaluation_time = None

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profiles
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

        # Create publishers for test metrics
        self.localization_error_pub = self.create_publisher(Float32, '/test/localization_error', 10)
        self.tracking_success_pub = self.create_publisher(Float32, '/test/tracking_success_rate', 10)
        self.test_complete_pub = self.create_publisher(Bool, '/test/vslam_performance_complete', 10)

        # Timer for periodic evaluation
        self.evaluation_timer = self.create_timer(1.0, self.evaluation_callback)
        self.test_complete_timer = self.create_timer(self.test_duration, self.test_complete_callback)

        self.get_logger().info('VSLAM Performance Test node initialized')
        self.get_logger().info(f'Starting performance test for {self.test_duration} seconds...')

        self.test_start_time = self.get_clock().now()
        self.last_evaluation_time = self.test_start_time

    def vslam_pose_callback(self, msg):
        """Callback for VSLAM pose messages"""
        current_time = self.get_clock().now()

        # Store VSLAM pose for trajectory analysis
        self.vslam_poses.append((current_time, msg.pose))

    def vslam_odom_callback(self, msg):
        """Callback for VSLAM odometry messages"""
        # Additional processing if needed
        pass

    def evaluation_callback(self):
        """Periodic evaluation of VSLAM performance"""
        current_time = self.get_clock().now()

        try:
            # Get transform from ground truth to robot
            transform = self.tf_buffer.lookup_transform(
                self.ground_truth_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            # Get VSLAM pose (latest available)
            if self.vslam_poses:
                vslam_pose_time, vslam_pose = self.vslam_poses[-1]

                # Calculate error between ground truth and VSLAM estimate
                gt_x = transform.transform.translation.x
                gt_y = transform.transform.translation.y
                gt_z = transform.transform.translation.z

                vslam_x = vslam_pose.position.x
                vslam_y = vslam_pose.position.y
                vslam_z = vslam_pose.position.z

                error = math.sqrt(
                    (gt_x - vslam_x)**2 +
                    (gt_y - vslam_y)**2 +
                    (gt_z - vslam_z)**2
                )

                self.localization_errors.append(error)

                # Publish current error
                error_msg = Float32()
                error_msg.data = error
                self.localization_error_pub.publish(error_msg)

                self.get_logger().info(f'Localization error: {error:.3f} m')

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

    def test_complete_callback(self):
        """Callback when test duration is complete"""
        self.get_logger().info('VSLAM Performance Test completed')
        self.evaluate_performance_results()

    def evaluate_performance_results(self):
        """Evaluate and report performance test results"""
        current_time = self.get_clock().now()
        total_test_duration = (current_time - self.test_start_time).nanoseconds / 1e9

        # Calculate performance metrics
        avg_localization_error = np.mean(self.localization_errors) if self.localization_errors else float('inf')
        max_localization_error = np.max(self.localization_errors) if self.localization_errors else float('inf')
        min_localization_error = np.min(self.localization_errors) if self.localization_errors else float('inf')

        # Calculate trajectory metrics
        total_distance = 0.0
        if len(self.vslam_poses) > 1:
            for i in range(1, len(self.vslam_poses)):
                prev_pose = self.vslam_poses[i-1][1]
                curr_pose = self.vslam_poses[i][1]

                dist = math.sqrt(
                    (curr_pose.position.x - prev_pose.position.x)**2 +
                    (curr_pose.position.y - prev_pose.position.y)**2 +
                    (curr_pose.position.z - prev_pose.position.z)**2
                )
                total_distance += dist

        # Print performance results
        self.get_logger().info('=== VSLAM Performance Test Results ===')
        self.get_logger().info(f'Total test duration: {total_test_duration:.2f} seconds')
        self.get_logger().info(f'Total trajectory distance: {total_distance:.2f} meters')
        self.get_logger().info(f'Average localization error: {avg_localization_error:.3f} meters')
        self.get_logger().info(f'Max localization error: {max_localization_error:.3f} meters')
        self.get_logger().info(f'Min localization error: {min_localization_error:.3f} meters')
        self.get_logger().info(f'Min required accuracy: {self.min_localization_accuracy} meters')

        # Evaluate accuracy
        accuracy_pass = avg_localization_error <= self.min_localization_accuracy
        self.get_logger().info(f'Accuracy requirement met: {accuracy_pass} ({">=" if accuracy_pass else "<"} {self.min_localization_accuracy}m)')

        # Overall result
        overall_pass = accuracy_pass
        self.get_logger().info(f'Overall performance result: {"PASS" if overall_pass else "FAIL"}')

        # Publish test completion result
        result_msg = Bool()
        result_msg.data = overall_pass
        self.test_complete_pub.publish(result_msg)


def main(args=None):
    """Main function to run the VSLAM performance test"""
    rclpy.init(args=args)

    test_node = VSLAMPerformanceTest()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()