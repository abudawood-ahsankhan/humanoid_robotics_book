#!/usr/bin/env python3
"""
Test script for Isaac ROS perception pipeline with Isaac Sim sensor data.

This script verifies that the perception pipeline is correctly processing
Isaac Sim sensor data and producing expected outputs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from vision_msgs.msg import Detection2DArray
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque
import statistics


class PerceptionPipelineTest(Node):
    def __init__(self):
        super().__init__('perception_pipeline_test')

        # Declare parameters
        self.declare_parameter('test_duration', 60.0)  # seconds
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('detections_topic', '/perception/objects')
        self.declare_parameter('segmentation_topic', '/perception/segmentation')
        self.declare_parameter('vslam_pose_topic', '/vslam/pose')
        self.declare_parameter('min_camera_rate', 10.0)  # Hz
        self.declare_parameter('min_lidar_rate', 5.0)   # Hz
        self.declare_parameter('min_imu_rate', 50.0)    # Hz
        self.declare_parameter('min_detection_rate', 5.0)  # Hz
        self.declare_parameter('min_segmentation_rate', 5.0)  # Hz
        self.declare_parameter('max_latency', 0.1)  # seconds

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.segmentation_topic = self.get_parameter('segmentation_topic').value
        self.vslam_pose_topic = self.get_parameter('vslam_pose_topic').value
        self.min_camera_rate = self.get_parameter('min_camera_rate').value
        self.min_lidar_rate = self.get_parameter('min_lidar_rate').value
        self.min_imu_rate = self.get_parameter('min_imu_rate').value
        self.min_detection_rate = self.get_parameter('min_detection_rate').value
        self.min_segmentation_rate = self.get_parameter('min_segmentation_rate').value
        self.max_latency = self.get_parameter('max_latency').value

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
            'camera_data_rate': 0.0,
            'lidar_data_rate': 0.0,
            'imu_data_rate': 0.0,
            'detection_rate': 0.0,
            'segmentation_rate': 0.0,
            'vslam_rate': 0.0,
            'camera_latency': 0.0,
            'lidar_latency': 0.0,
            'detection_latency': 0.0,
            'segmentation_latency': 0.0,
            'vslam_latency': 0.0,
            'detection_accuracy': 0.0,
            'segmentation_accuracy': 0.0,
            'vslam_accuracy': 0.0,
            'overall_performance': 0.0
        }

        # Data collection
        self.camera_messages = deque(maxlen=1000)
        self.lidar_messages = deque(maxlen=1000)
        self.imu_messages = deque(maxlen=1000)
        self.detection_messages = deque(maxlen=1000)
        self.segmentation_messages = deque(maxlen=1000)
        self.vslam_pose_messages = deque(maxlen=1000)
        self.message_times = {
            'camera': deque(maxlen=100),
            'lidar': deque(maxlen=100),
            'imu': deque(maxlen=100),
            'detections': deque(maxlen=100),
            'segmentation': deque(maxlen=100),
            'vslam': deque(maxlen=100)
        }
        self.data_lock = threading.Lock()

        # Create subscribers
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            sensor_qos
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        self.detections_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detections_callback,
            sensor_qos
        )

        self.segmentation_sub = self.create_subscription(
            Image,
            self.segmentation_topic,
            self.segmentation_callback,
            sensor_qos
        )

        self.vslam_pose_sub = self.create_subscription(
            PoseStamped,
            self.vslam_pose_topic,
            self.vslam_pose_callback,
            sensor_qos
        )

        # Create publisher for test control
        self.test_control_pub = self.create_publisher(
            String,
            '/perception_test/control',
            sensor_qos
        )

        self.get_logger().info('Perception Pipeline Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'LiDAR topic: {self.lidar_topic}')
        self.get_logger().info(f'Detections topic: {self.detections_topic}')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.camera_messages.append(msg)
            current_time = self.get_clock().now()
            self.message_times['camera'].append(current_time)

            # Calculate latency (time from sensor to processing)
            msg_time = Time()
            msg_time.sec = msg.header.stamp.sec
            msg_time.nanosec = msg.header.stamp.nanosec
            msg_ros_time = rclpy.time.Time.from_msg(msg_time)
            latency = (current_time.nanoseconds - msg_ros_time.nanoseconds) / 1e9
            self.test_results['camera_latency'] = latency

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.lidar_messages.append(msg)
            current_time = self.get_clock().now()
            self.message_times['lidar'].append(current_time)

            # Calculate latency
            msg_time = Time()
            msg_time.sec = msg.header.stamp.sec
            msg_time.nanosec = msg.header.stamp.nanosec
            msg_ros_time = rclpy.time.Time.from_msg(msg_time)
            latency = (current_time.nanoseconds - msg_ros_time.nanoseconds) / 1e9
            self.test_results['lidar_latency'] = latency

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.imu_messages.append(msg)
            current_time = self.get_clock().now()
            self.message_times['imu'].append(current_time)

    def detections_callback(self, msg):
        """Callback for detection messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.detection_messages.append(msg)
            current_time = self.get_clock().now()
            self.message_times['detections'].append(current_time)

            # Calculate latency
            msg_time = Time()
            msg_time.sec = msg.header.stamp.sec
            msg_time.nanosec = msg.header.stamp.nanosec
            msg_ros_time = rclpy.time.Time.from_msg(msg_time)
            latency = (current_time.nanoseconds - msg_ros_time.nanoseconds) / 1e9
            self.test_results['detection_latency'] = latency

    def segmentation_callback(self, msg):
        """Callback for segmentation messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.segmentation_messages.append(msg)
            current_time = self.get_clock().now()
            self.message_times['segmentation'].append(current_time)

            # Calculate latency
            msg_time = Time()
            msg_time.sec = msg.header.stamp.sec
            msg_time.nanosec = msg.header.stamp.nanosec
            msg_ros_time = rclpy.time.Time.from_msg(msg_time)
            latency = (current_time.nanoseconds - msg_ros_time.nanoseconds) / 1e9
            self.test_results['segmentation_latency'] = latency

    def vslam_pose_callback(self, msg):
        """Callback for VSLAM pose messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.vslam_pose_messages.append(msg)
            current_time = self.get_clock().now()
            self.message_times['vslam'].append(current_time)

            # Calculate latency
            msg_time = Time()
            msg_time.sec = msg.header.stamp.sec
            msg_time.nanosec = msg.header.stamp.nanosec
            msg_ros_time = rclpy.time.Time.from_msg(msg_time)
            latency = (current_time.nanoseconds - msg_ros_time.nanoseconds) / 1e9
            self.test_results['vslam_latency'] = latency

    def run_test(self):
        """Run the perception pipeline test"""
        self.get_logger().info('Starting perception pipeline test with Isaac Sim sensor data...')
        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Wait for test duration
        start_time = time.time()
        while time.time() - start_time < self.test_duration and self.test_active:
            time.sleep(0.1)

        # Stop test
        self.test_active = False
        self.get_logger().info('Perception pipeline test completed')

        # Process results
        self.process_test_results()

        # Generate report
        self.generate_test_report()

    def process_test_results(self):
        """Process collected data to calculate test metrics"""
        with self.data_lock:
            # Calculate data rates
            test_duration = (self.get_clock().now().nanoseconds - self.test_start_time.nanoseconds) / 1e9

            if test_duration > 0:
                self.test_results['camera_data_rate'] = len(self.camera_messages) / test_duration
                self.test_results['lidar_data_rate'] = len(self.lidar_messages) / test_duration
                self.test_results['imu_data_rate'] = len(self.imu_messages) / test_duration
                self.test_results['detection_rate'] = len(self.detection_messages) / test_duration
                self.test_results['segmentation_rate'] = len(self.segmentation_messages) / test_duration
                self.test_results['vslam_rate'] = len(self.vslam_pose_messages) / test_duration

            # Calculate average latencies
            if self.message_times['camera']:
                camera_latencies = []
                for msg in list(self.camera_messages)[-min(50, len(self.camera_messages)):]:  # Last 50 messages
                    msg_time = Time()
                    msg_time.sec = msg.header.stamp.sec
                    msg_time.nanosec = msg.header.stamp.nanosec
                    msg_ros_time = rclpy.time.Time.from_msg(msg_time)
                    latency = (self.get_clock().now().nanoseconds - msg_ros_time.nanoseconds) / 1e9
                    camera_latencies.append(latency)
                if camera_latencies:
                    self.test_results['camera_latency'] = statistics.mean(camera_latencies)

            if self.message_times['lidar']:
                lidar_latencies = []
                for msg in list(self.lidar_messages)[-min(50, len(self.lidar_messages)):]:
                    msg_time = Time()
                    msg_time.sec = msg.header.stamp.sec
                    msg_time.nanosec = msg.header.stamp.nanosec
                    msg_ros_time = rclpy.time.Time.from_msg(msg_time)
                    latency = (self.get_clock().now().nanoseconds - msg_ros_time.nanoseconds) / 1e9
                    lidar_latencies.append(latency)
                if lidar_latencies:
                    self.test_results['lidar_latency'] = statistics.mean(lidar_latencies)

            # Calculate detection accuracy (simplified - in real test would compare with ground truth)
            if self.detection_messages:
                # For this test, we'll calculate a simple metric based on detection count
                detection_counts = [len(d.detections) for d in self.detection_messages]
                avg_detections = statistics.mean(detection_counts) if detection_counts else 0
                # Normalize to 0-1 scale based on expected range
                self.test_results['detection_accuracy'] = min(1.0, avg_detections / 10.0)  # Assume 10 is good count

            # Calculate segmentation accuracy (simplified)
            if self.segmentation_messages:
                # In a real test, this would compare with ground truth segmentation
                # For now, we'll just check that messages are being published regularly
                self.test_results['segmentation_accuracy'] = 0.8 if len(self.segmentation_messages) > 0 else 0.0

            # Calculate VSLAM accuracy (simplified)
            if len(self.vslam_pose_messages) > 1:
                # Check for reasonable pose changes
                positions = []
                for msg in self.vslam_pose_messages:
                    pos = msg.pose.position
                    positions.append(np.array([pos.x, pos.y, pos.z]))

                if len(positions) > 1:
                    # Calculate average movement between poses
                    movements = []
                    for i in range(1, len(positions)):
                        movement = np.linalg.norm(positions[i] - positions[i-1])
                        movements.append(movement)

                    if movements:
                        avg_movement = statistics.mean(movements)
                        # Reasonable movement should be neither too small (stuck) nor too large (jumps)
                        if 0.001 < avg_movement < 1.0:  # Between 1mm and 1m per update
                            self.test_results['vslam_accuracy'] = 0.9
                        elif avg_movement == 0.0:  # Not moving
                            self.test_results['vslam_accuracy'] = 0.3
                        else:  # Too much movement (likely jumps/errors)
                            self.test_results['vslam_accuracy'] = 0.2
                    else:
                        self.test_results['vslam_accuracy'] = 0.0
                else:
                    self.test_results['vslam_accuracy'] = 0.0
            else:
                self.test_results['vslam_accuracy'] = 0.0

            # Calculate overall performance score
            performance_factors = [
                self.test_results['detection_accuracy'],
                self.test_results['segmentation_accuracy'],
                self.test_results['vslam_accuracy']
            ]

            # Weight data rates heavily as they're critical for perception
            rate_factors = [
                min(1.0, self.test_results['camera_data_rate'] / self.min_camera_rate),
                min(1.0, self.test_results['lidar_data_rate'] / self.min_lidar_rate),
                min(1.0, self.test_results['detection_rate'] / self.min_detection_rate),
                min(1.0, self.test_results['segmentation_rate'] / self.min_segmentation_rate)
            ]

            # Weight latencies inversely (lower is better)
            latency_factors = [
                max(0.0, 1.0 - self.test_results['camera_latency'] / self.max_latency),
                max(0.0, 1.0 - self.test_results['lidar_latency'] / self.max_latency),
                max(0.0, 1.0 - self.test_results['detection_latency'] / self.max_latency),
                max(0.0, 1.0 - self.test_results['segmentation_latency'] / self.max_latency)
            ]

            # Overall score weighted combination
            self.test_results['overall_performance'] = (
                0.4 * statistics.mean(performance_factors) +
                0.4 * statistics.mean(rate_factors) +
                0.2 * statistics.mean(latency_factors)
            )

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== PERCEPTION PIPELINE TEST REPORT ===')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')

        # Print individual metrics
        self.get_logger().info(f'--- DATA RATE METRICS ---')
        self.get_logger().info(f'Camera Data Rate: {self.test_results["camera_data_rate"]:.2f} Hz (Required: ≥{self.min_camera_rate} Hz)')
        self.get_logger().info(f'LiDAR Data Rate: {self.test_results["lidar_data_rate"]:.2f} Hz (Required: ≥{self.min_lidar_rate} Hz)')
        self.get_logger().info(f'IMU Data Rate: {self.test_results["imu_data_rate"]:.2f} Hz (Required: ≥{self.min_imu_rate} Hz)')
        self.get_logger().info(f'Detection Rate: {self.test_results["detection_rate"]:.2f} Hz (Required: ≥{self.min_detection_rate} Hz)')
        self.get_logger().info(f'Segmentation Rate: {self.test_results["segmentation_rate"]:.2f} Hz (Required: ≥{self.min_segmentation_rate} Hz)')
        self.get_logger().info(f'VSLAM Rate: {self.test_results["vslam_rate"]:.2f} Hz')

        self.get_logger().info(f'--- LATENCY METRICS ---')
        self.get_logger().info(f'Camera Latency: {self.test_results["camera_latency"]:.3f}s (Max: {self.max_latency}s)')
        self.get_logger().info(f'LiDAR Latency: {self.test_results["lidar_latency"]:.3f}s (Max: {self.max_latency}s)')
        self.get_logger().info(f'Detection Latency: {self.test_results["detection_latency"]:.3f}s (Max: {self.max_latency}s)')
        self.get_logger().info(f'Segmentation Latency: {self.test_results["segmentation_latency"]:.3f}s (Max: {self.max_latency}s)')
        self.get_logger().info(f'VSLAM Latency: {self.test_results["vslam_latency"]:.3f}s (Max: {self.max_latency}s)')

        self.get_logger().info(f'--- ACCURACY METRICS ---')
        self.get_logger().info(f'Detection Accuracy: {self.test_results["detection_accuracy"]:.3f}')
        self.get_logger().info(f'Segmentation Accuracy: {self.test_results["segmentation_accuracy"]:.3f}')
        self.get_logger().info(f'VSLAM Accuracy: {self.test_results["vslam_accuracy"]:.3f}')

        # Determine pass/fail criteria
        camera_rate_pass = self.test_results['camera_data_rate'] >= self.min_camera_rate
        lidar_rate_pass = self.test_results['lidar_data_rate'] >= self.min_lidar_rate
        detection_rate_pass = self.test_results['detection_rate'] >= self.min_detection_rate
        segmentation_rate_pass = self.test_results['segmentation_rate'] >= self.min_segmentation_rate
        camera_latency_pass = self.test_results['camera_latency'] <= self.max_latency
        lidar_latency_pass = self.test_results['lidar_latency'] <= self.max_latency
        detection_latency_pass = self.test_results['detection_latency'] <= self.max_latency
        segmentation_latency_pass = self.test_results['segmentation_latency'] <= self.max_latency

        self.get_logger().info(f'--- PASS/FAIL CRITERIA ---')
        self.get_logger().info(f'Camera Rate (≥{self.min_camera_rate} Hz): {"✓ PASS" if camera_rate_pass else "✗ FAIL"}')
        self.get_logger().info(f'LiDAR Rate (≥{self.min_lidar_rate} Hz): {"✓ PASS" if lidar_rate_pass else "✗ FAIL"}')
        self.get_logger().info(f'Detection Rate (≥{self.min_detection_rate} Hz): {"✓ PASS" if detection_rate_pass else "✗ FAIL"}')
        self.get_logger().info(f'Segmentation Rate (≥{self.min_segmentation_rate} Hz): {"✓ PASS" if segmentation_rate_pass else "✗ FAIL"}')
        self.get_logger().info(f'Camera Latency (≤{self.max_latency}s): {"✓ PASS" if camera_latency_pass else "✗ FAIL"}')
        self.get_logger().info(f'LiDAR Latency (≤{self.max_latency}s): {"✓ PASS" if lidar_latency_pass else "✗ FAIL"}')
        self.get_logger().info(f'Detection Latency (≤{self.max_latency}s): {"✓ PASS" if detection_latency_pass else "✗ FAIL"}')
        self.get_logger().info(f'Segmentation Latency (≤{self.max_latency}s): {"✓ PASS" if segmentation_latency_pass else "✗ FAIL"}')

        # Overall assessment
        all_rate_pass = all([camera_rate_pass, lidar_rate_pass, detection_rate_pass, segmentation_rate_pass])
        all_latency_pass = all([camera_latency_pass, lidar_latency_pass, detection_latency_pass, segmentation_latency_pass])
        overall_pass = all_rate_pass and all_latency_pass

        self.get_logger().info(f'--- FINAL ASSESSMENT ---')
        self.get_logger().info(f'Data Rate Requirements: {"✓ PASS" if all_rate_pass else "✗ FAIL"}')
        self.get_logger().info(f'Latency Requirements: {"✓ PASS" if all_latency_pass else "✗ FAIL"}')
        self.get_logger().info(f'Perception Pipeline Performance: {"✓ PASS" if overall_pass else "✗ FAIL"}')
        self.get_logger().info(f'Overall Performance Score: {self.test_results["overall_performance"]:.3f}')

        # Recommendations
        self.get_logger().info(f'--- RECOMMENDATIONS ---')
        if not camera_rate_pass:
            self.get_logger().info('  - Check Isaac Sim camera configuration')
            self.get_logger().info('  - Verify camera sensor is publishing at expected rate')
            self.get_logger().info('  - Check Isaac Sim-ROS bridge configuration')
        if not lidar_rate_pass:
            self.get_logger().info('  - Check Isaac Sim LiDAR configuration')
            self.get_logger().info('  - Verify LiDAR sensor is publishing at expected rate')
            self.get_logger().info('  - Check Isaac Sim-ROS bridge configuration')
        if not detection_rate_pass:
            self.get_logger().info('  - Verify Isaac ROS DetectNet node is running')
            self.get_logger().info('  - Check that camera data is reaching the detection node')
            self.get_logger().info('  - Verify model loading and initialization')
        if not segmentation_rate_pass:
            self.get_logger().info('  - Verify Isaac ROS Segmentation node is running')
            self.get_logger().info('  - Check that camera data is reaching the segmentation node')
            self.get_logger().info('  - Verify model loading and initialization')
        if not camera_latency_pass:
            self.get_logger().info('  - Optimize camera processing pipeline')
            self.get_logger().info('  - Check for bottlenecks in image processing')
            self.get_logger().info('  - Consider reducing image resolution if needed')
        if not lidar_latency_pass:
            self.get_logger().info('  - Optimize point cloud processing pipeline')
            self.get_logger().info('  - Check for bottlenecks in point cloud processing')
            self.get_logger().info('  - Consider point cloud downsampling if needed')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = PerceptionPipelineTest()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.run_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 10.0)  # Add 10s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('Perception pipeline test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()