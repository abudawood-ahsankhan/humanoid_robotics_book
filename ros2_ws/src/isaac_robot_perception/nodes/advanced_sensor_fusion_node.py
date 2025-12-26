#!/usr/bin/env python3
"""
Isaac ROS Advanced Sensor Fusion Node

This node implements advanced sensor fusion combining camera, LiDAR, and IMU data
for enhanced perception and state estimation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R
from message_filters import ApproximateTimeSynchronizer, Subscriber


class AdvancedSensorFusionNode(Node):
    def __init__(self):
        super().__init__('advanced_sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('detections_topic', '/perception/objects')
        self.declare_parameter('fused_output_topic', '/perception/fused_data')
        self.declare_parameter('state_output_topic', '/perception/fused_state')
        self.declare_parameter('sync_tolerance', 0.1)
        self.declare_parameter('processing_frequency', 30.0)
        self.declare_parameter('enable_camera_fusion', True)
        self.declare_parameter('enable_lidar_fusion', True)
        self.declare_parameter('enable_imu_fusion', True)

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.fused_output_topic = self.get_parameter('fused_output_topic').value
        self.state_output_topic = self.get_parameter('state_output_topic').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        self.processing_frequency = self.get_parameter('processing_frequency').value
        self.enable_camera_fusion = self.get_parameter('enable_camera_fusion').value
        self.enable_lidar_fusion = self.get_parameter('enable_lidar_fusion').value
        self.enable_imu_fusion = self.get_parameter('enable_imu_fusion').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize sensor data buffers
        self.camera_buffer = deque(maxlen=10)
        self.lidar_buffer = deque(maxlen=10)
        self.imu_buffer = deque(maxlen=50)  # More IMU data for filtering
        self.detection_buffer = deque(maxlen=10)
        self.data_lock = threading.Lock()

        # State estimation variables
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w (quaternion)
        self.angular_velocity = np.array([0.0, 0.0, 0.0])   # x, y, z
        self.linear_acceleration = np.array([0.0, 0.0, 0.0]) # x, y, z
        self.position = np.array([0.0, 0.0, 0.0])           # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])           # x, y, z

        # Time tracking
        self.last_update_time = self.get_clock().now()

        # Create subscribers
        self.camera_sub = Subscriber(self, Image, self.camera_topic, qos_profile=sensor_qos)
        self.lidar_sub = Subscriber(self, PointCloud2, self.lidar_topic, qos_profile=sensor_qos)
        self.imu_sub = Subscriber(self, Imu, self.imu_topic, qos_profile=sensor_qos)
        self.detection_sub = Subscriber(self, Detection2DArray, self.detections_topic, qos_profile=sensor_qos)

        # Create approximate time synchronizer
        self.ats = ApproximateTimeSynchronizer(
            [self.camera_sub, self.lidar_sub, self.imu_sub, self.detection_sub],
            queue_size=10,
            slop=self.sync_tolerance
        )
        self.ats.registerCallback(self.synchronized_callback)

        # Create publishers
        self.fused_output_pub = self.create_publisher(
            PointCloud2,  # Using PointCloud2 to represent fused sensor data
            self.fused_output_topic,
            sensor_qos
        )

        self.state_pub = self.create_publisher(
            Imu,  # Using Imu msg for state (contains pose and twist)
            self.state_output_topic,
            sensor_qos
        )

        # Create timer for fusion processing
        self.fusion_timer = self.create_timer(1.0 / self.processing_frequency, self.perform_advanced_fusion)

        self.get_logger().info('Advanced Sensor Fusion Node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'LiDAR topic: {self.lidar_topic}')
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')

    def synchronized_callback(self, camera_msg, lidar_msg, imu_msg, detection_msg):
        """Callback for synchronized sensor data"""
        with self.data_lock:
            # Store synchronized data
            self.camera_buffer.append(camera_msg)
            self.lidar_buffer.append(lidar_msg)
            self.imu_buffer.append(imu_msg)
            self.detection_buffer.append(detection_msg)

            # Extract IMU data for state estimation
            self.orientation = np.array([
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w
            ])

            self.angular_velocity = np.array([
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            ])

            self.linear_acceleration = np.array([
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            ])

    def perform_advanced_fusion(self):
        """Perform advanced sensor fusion to estimate state and create fused representation"""
        if len(self.imu_buffer) == 0:
            return

        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_update_time.nanoseconds) / 1e9
        self.last_update_time = current_time

        if dt <= 0:
            return

        with self.data_lock:
            # Get the most recent IMU data for fusion
            latest_imu = self.imu_buffer[-1]

            # Perform sensor fusion using complementary filter approach
            fused_orientation = self.orientation.copy()
            fused_angular_velocity = self.angular_velocity.copy()
            fused_linear_acceleration = self.linear_acceleration.copy()

            # Integrate angular velocity to update orientation
            angular_speed = np.linalg.norm(self.angular_velocity)
            if angular_speed > 1e-6:  # Avoid division by zero
                axis = self.angular_velocity / angular_speed
                angle = angular_speed * dt

                # Create rotation quaternion from axis-angle
                axis_quat = np.array([np.sin(angle/2) * axis[0],
                                     np.sin(angle/2) * axis[1],
                                     np.sin(angle/2) * axis[2],
                                     np.cos(angle/2)])

                # Apply rotation to current orientation
                fused_orientation = self.quaternion_multiply(self.orientation, axis_quat)
                # Normalize quaternion
                fused_orientation = fused_orientation / np.linalg.norm(fused_orientation)

            # Update position and velocity from acceleration
            # Apply rotation to transform acceleration to world frame
            rot_matrix = R.from_quat(fused_orientation).as_matrix()
            world_acceleration = rot_matrix @ self.linear_acceleration

            # Update velocity and position
            self.velocity += world_acceleration * dt
            self.position += self.velocity * dt

        # Create and publish fused state message
        fused_state_msg = Imu()
        fused_state_msg.header.stamp = current_time.to_msg()
        fused_state_msg.header.frame_id = latest_imu.header.frame_id

        # Set fused orientation
        fused_state_msg.orientation.x = float(fused_orientation[0])
        fused_state_msg.orientation.y = float(fused_orientation[1])
        fused_state_msg.orientation.z = float(fused_orientation[2])
        fused_state_msg.orientation.w = float(fused_orientation[3])

        # Set fused angular velocity
        fused_state_msg.angular_velocity.x = float(fused_angular_velocity[0])
        fused_state_msg.angular_velocity.y = float(fused_angular_velocity[1])
        fused_state_msg.angular_velocity.z = float(fused_angular_velocity[2])

        # Set fused linear acceleration
        fused_state_msg.linear_acceleration.x = float(fused_linear_acceleration[0])
        fused_state_msg.linear_acceleration.y = float(fused_linear_acceleration[1])
        fused_state_msg.linear_acceleration.z = float(fused_linear_acceleration[2])

        # Set covariance matrices (set to zero for now)
        fused_state_msg.orientation_covariance = [0.0] * 9
        fused_state_msg.angular_velocity_covariance = [0.0] * 9
        fused_state_msg.linear_acceleration_covariance = [0.0] * 9

        self.state_pub.publish(fused_state_msg)

        # Create fused output combining sensor data
        # For this example, we'll create a simplified fused representation
        if len(self.lidar_buffer) > 0:
            # Use the latest LiDAR data as the base for fused output
            latest_lidar = self.lidar_buffer[-1]
            self.fused_output_pub.publish(latest_lidar)

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([x, y, z, w])

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    fusion_node = AdvancedSensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Advanced Sensor Fusion Node interrupted by user')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()