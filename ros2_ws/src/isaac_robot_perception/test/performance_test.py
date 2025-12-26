#!/usr/bin/env python3
"""
Performance Test for Isaac ROS Perception Pipeline

This script tests the real-time processing performance of the Isaac ROS perception pipeline
with hardware acceleration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from collections import deque
import psutil
import GPUtil


class PerformanceTestNode(Node):
    def __init__(self):
        super().__init__('performance_test_node')

        # Declare parameters
        self.declare_parameter('test_duration', 30.0)  # seconds
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('detections_topic', '/perception/objects')
        self.declare_parameter('expected_camera_frequency', 30.0)  # Hz
        self.declare_parameter('expected_lidar_frequency', 10.0)   # Hz
        self.declare_parameter('expected_imu_frequency', 100.0)   # Hz
        self.declare_parameter('latency_threshold', 0.05)         # seconds

        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.expected_camera_frequency = self.get_parameter('expected_camera_frequency').value
        self.expected_lidar_frequency = self.get_parameter('expected_lidar_frequency').value
        self.expected_imu_frequency = self.get_parameter('expected_imu_frequency').value
        self.latency_threshold = self.get_parameter('latency_threshold').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize test variables
        self.test_start_time = None
        self.test_active = False
        self.results = {
            'camera': {'timestamps': [], 'frequencies': [], 'latencies': []},
            'lidar': {'timestamps': [], 'frequencies': [], 'latencies': []},
            'imu': {'timestamps': [], 'frequencies': [], 'latencies': []},
            'detections': {'timestamps': [], 'frequencies': [], 'latencies': []}
        }

        # Initialize performance monitoring
        self.cpu_usage = deque(maxlen=100)
        self.gpu_usage = deque(maxlen=100)
        self.memory_usage = deque(maxlen=100)

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

        # Create timer for performance monitoring
        self.monitor_timer = self.create_timer(0.1, self.monitor_performance)

        self.get_logger().info('Performance Test Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'Expected frequencies - Camera: {self.expected_camera_frequency} Hz, LiDAR: {self.expected_lidar_frequency} Hz, IMU: {self.expected_imu_frequency} Hz')

    def start_test(self):
        """Start the performance test"""
        self.get_logger().info('Starting performance test...')
        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Wait for test duration
        time.sleep(self.test_duration)

        # Stop test
        self.test_active = False
        self.get_logger().info('Performance test completed')

        # Generate and print results
        self.generate_test_report()

    def camera_callback(self, msg):
        """Callback for camera messages"""
        if not self.test_active:
            return

        current_time = self.get_clock().now()
        timestamp = current_time.nanoseconds / 1e9
        msg_timestamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # Calculate frequency and latency
        self.results['camera']['timestamps'].append(timestamp)
        if len(self.results['camera']['timestamps']) > 1:
            time_diff = timestamp - self.results['camera']['timestamps'][-2]
            frequency = 1.0 / time_diff if time_diff > 0 else 0
            self.results['camera']['frequencies'].append(frequency)

        latency = timestamp - msg_timestamp
        self.results['camera']['latencies'].append(latency)

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        if not self.test_active:
            return

        current_time = self.get_clock().now()
        timestamp = current_time.nanoseconds / 1e9
        msg_timestamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # Calculate frequency and latency
        self.results['lidar']['timestamps'].append(timestamp)
        if len(self.results['lidar']['timestamps']) > 1:
            time_diff = timestamp - self.results['lidar']['timestamps'][-2]
            frequency = 1.0 / time_diff if time_diff > 0 else 0
            self.results['lidar']['frequencies'].append(frequency)

        latency = timestamp - msg_timestamp
        self.results['lidar']['latencies'].append(latency)

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        if not self.test_active:
            return

        current_time = self.get_clock().now()
        timestamp = current_time.nanoseconds / 1e9
        msg_timestamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # Calculate frequency and latency
        self.results['imu']['timestamps'].append(timestamp)
        if len(self.results['imu']['timestamps']) > 1:
            time_diff = timestamp - self.results['imu']['timestamps'][-2]
            frequency = 1.0 / time_diff if time_diff > 0 else 0
            self.results['imu']['frequencies'].append(frequency)

        latency = timestamp - msg_timestamp
        self.results['imu']['latencies'].append(latency)

    def detections_callback(self, msg):
        """Callback for detection messages"""
        if not self.test_active:
            return

        current_time = self.get_clock().now()
        timestamp = current_time.nanoseconds / 1e9
        msg_timestamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # Calculate frequency and latency
        self.results['detections']['timestamps'].append(timestamp)
        if len(self.results['detections']['timestamps']) > 1:
            time_diff = timestamp - self.results['detections']['timestamps'][-2]
            frequency = 1.0 / time_diff if time_diff > 0 else 0
            self.results['detections']['frequencies'].append(frequency)

        latency = timestamp - msg_timestamp
        self.results['detections']['latencies'].append(latency)

    def monitor_performance(self):
        """Monitor system performance during the test"""
        if not self.test_active:
            return

        # Monitor CPU usage
        cpu_percent = psutil.cpu_percent()
        self.cpu_usage.append(cpu_percent)

        # Monitor memory usage
        memory_percent = psutil.virtual_memory().percent
        self.memory_usage.append(memory_percent)

        # Monitor GPU usage (if available)
        try:
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu_percent = gpus[0].load * 100
                self.gpu_usage.append(gpu_percent)
            else:
                self.gpu_usage.append(0.0)
        except:
            self.gpu_usage.append(0.0)

    def generate_test_report(self):
        """Generate and print the test report"""
        self.get_logger().info('=== PERFORMANCE TEST REPORT ===')

        # Calculate and report results for each sensor
        for sensor_type, data in self.results.items():
            if data['frequencies']:
                avg_frequency = np.mean(data['frequencies'])
                min_frequency = np.min(data['frequencies'])
                max_frequency = np.max(data['frequencies'])

                avg_latency = np.mean(data['latencies'])
                max_latency = np.max(data['latencies'])
                min_latency = np.min(data['latencies'])

                # Calculate performance metrics
                expected_freq = getattr(self, f'expected_{sensor_type}_frequency', 0)
                frequency_met = avg_frequency >= expected_freq * 0.9  # Allow 10% tolerance

                latency_met = avg_latency <= self.latency_threshold

                self.get_logger().info(f'--- {sensor_type.upper()} ---')
                self.get_logger().info(f'  Average frequency: {avg_frequency:.2f} Hz (Expected: {expected_freq:.2f} Hz)')
                self.get_logger().info(f'  Frequency range: {min_frequency:.2f} - {max_frequency:.2f} Hz')
                self.get_logger().info(f'  Frequency requirement met: {"✓" if frequency_met else "✗"}')

                self.get_logger().info(f'  Average latency: {avg_latency:.4f} s (Threshold: {self.latency_threshold:.4f} s)')
                self.get_logger().info(f'  Latency range: {min_latency:.4f} - {max_latency:.4f} s')
                self.get_logger().info(f'  Latency requirement met: {"✓" if latency_met else "✗"}')

                self.get_logger().info(f'  Total messages received: {len(data["timestamps"])}')
            else:
                self.get_logger().info(f'--- {sensor_type.upper()} ---')
                self.get_logger().info('  No data received during test')

        # Report system performance
        if self.cpu_usage:
            avg_cpu = np.mean(self.cpu_usage)
            max_cpu = np.max(self.cpu_usage)
            self.get_logger().info(f'--- SYSTEM PERFORMANCE ---')
            self.get_logger().info(f'  Average CPU usage: {avg_cpu:.2f}%')
            self.get_logger().info(f'  Maximum CPU usage: {max_cpu:.2f}%')

        if self.gpu_usage:
            avg_gpu = np.mean(self.gpu_usage)
            max_gpu = np.max(self.gpu_usage)
            self.get_logger().info(f'  Average GPU usage: {avg_gpu:.2f}%')
            self.get_logger().info(f'  Maximum GPU usage: {max_gpu:.2f}%')

        if self.memory_usage:
            avg_memory = np.mean(self.memory_usage)
            max_memory = np.max(self.memory_usage)
            self.get_logger().info(f'  Average memory usage: {avg_memory:.2f}%')
            self.get_logger().info(f'  Maximum memory usage: {max_memory:.2f}%')

        # Overall assessment
        self.get_logger().info(f'--- OVERALL ASSESSMENT ---')
        all_freq_ok = all(
            np.mean(data['frequencies']) >= getattr(self, f'expected_{sensor_type}_frequency', 0) * 0.9
            for sensor_type, data in self.results.items()
            if data['frequencies']
        )

        all_latency_ok = all(
            np.mean(data['latencies']) <= self.latency_threshold
            for data in self.results.values()
            if data['latencies']
        )

        self.get_logger().info(f'  Real-time performance achieved: {"✓" if all_freq_ok else "✗"}')
        self.get_logger().info(f'  Latency requirements met: {"✓" if all_latency_ok else "✗"}')
        self.get_logger().info(f'  Hardware acceleration effective: {"✓" if np.mean(self.gpu_usage) > 10.0 else "✗"}')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    test_node = PerformanceTestNode()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=test_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(test_node)

        # Wait for test to complete
        test_thread.join(timeout=test_node.test_duration + 5.0)  # Add 5s buffer
    except KeyboardInterrupt:
        test_node.get_logger().info('Performance test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()