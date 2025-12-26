#!/usr/bin/env python3
"""
Isaac ROS Sensor Fusion Node

This node integrates IMU data with other sensors (camera, LiDAR)
for improved perception and state estimation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2, Image
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('fused_imu_topic', '/perception/imu/fused')
        self.declare_parameter('state_topic', '/perception/state')
        self.declare_parameter('enable_camera_fusion', True)
        self.declare_parameter('enable_lidar_fusion', True)
        self.declare_parameter('processing_frequency', 100.0)
        self.declare_parameter('sync_tolerance', 0.05)

        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.fused_imu_topic = self.get_parameter('fused_imu_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.enable_camera_fusion = self.get_parameter('enable_camera_fusion').value
        self.enable_lidar_fusion = self.get_parameter('enable_lidar_fusion').value
        self.processing_frequency = self.get_parameter('processing_frequency').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize sensor data buffers
        self.imu_data = None
        self.camera_data = None
        self.lidar_data = None
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
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        if self.enable_camera_fusion:
            self.camera_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                sensor_qos
            )

        if self.enable_lidar_fusion:
            self.lidar_sub = self.create_subscription(
                PointCloud2,
                self.lidar_topic,
                self.lidar_callback,
                sensor_qos
            )

        # Create publishers
        self.fused_imu_pub = self.create_publisher(
            Imu,
            self.fused_imu_topic,
            sensor_qos
        )

        self.state_pub = self.create_publisher(
            Imu,  # Using Imu msg for state (contains pose and twist)
            self.state_topic,
            sensor_qos
        )

        # Create timer for fusion processing
        self.fusion_timer = self.create_timer(1.0 / self.processing_frequency, self.perform_fusion)

        self.get_logger().info('Sensor Fusion Node initialized')
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        with self.data_lock:
            # Extract IMU data
            self.orientation = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            self.angular_velocity = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            self.linear_acceleration = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            self.imu_data = msg
            self.get_logger().debug(f'Received IMU data: orientation=({self.orientation})')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        with self.data_lock:
            self.camera_data = msg
            self.get_logger().debug('Received camera data')

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        with self.data_lock:
            self.lidar_data = msg
            self.get_logger().debug(f'Received LiDAR data with {msg.width * msg.height} points')

    def perform_fusion(self):
        """Perform sensor fusion to estimate state"""
        if self.imu_data is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_update_time.nanoseconds) / 1e9
        self.last_update_time = current_time

        if dt <= 0:
            return

        with self.data_lock:
            # Perform complementary filter fusion
            # This is a simplified version - in a real implementation,
            # this would use more sophisticated fusion algorithms
            fused_orientation = self.orientation.copy()
            fused_angular_velocity = self.angular_velocity.copy()
            fused_linear_acceleration = self.linear_acceleration.copy()

            # Integrate angular velocity to update orientation
            # (Simplified integration - in practice, would use more accurate methods)
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

            # Update position and velocity from acceleration (simplified)
            # Apply rotation to transform acceleration to world frame
            rot_matrix = R.from_quat(fused_orientation).as_matrix()
            world_acceleration = rot_matrix @ self.linear_acceleration

            # Update velocity and position
            self.velocity += world_acceleration * dt
            self.position += self.velocity * dt

        # Create and publish fused IMU message
        fused_imu_msg = Imu()
        fused_imu_msg.header.stamp = current_time.to_msg()
        fused_imu_msg.header.frame_id = self.imu_data.header.frame_id

        # Set fused orientation
        fused_imu_msg.orientation.x = float(fused_orientation[0])
        fused_imu_msg.orientation.y = float(fused_orientation[1])
        fused_imu_msg.orientation.z = float(fused_orientation[2])
        fused_imu_msg.orientation.w = float(fused_orientation[3])

        # Set fused angular velocity
        fused_imu_msg.angular_velocity.x = float(fused_angular_velocity[0])
        fused_imu_msg.angular_velocity.y = float(fused_angular_velocity[1])
        fused_imu_msg.angular_velocity.z = float(fused_angular_velocity[2])

        # Set fused linear acceleration
        fused_imu_msg.linear_acceleration.x = float(fused_linear_acceleration[0])
        fused_imu_msg.linear_acceleration.y = float(fused_linear_acceleration[1])
        fused_imu_msg.linear_acceleration.z = float(fused_linear_acceleration[2])

        # Set covariance matrices (set to zero for now)
        fused_imu_msg.orientation_covariance = [0.0] * 9
        fused_imu_msg.angular_velocity_covariance = [0.0] * 9
        fused_imu_msg.linear_acceleration_covariance = [0.0] * 9

        self.fused_imu_pub.publish(fused_imu_msg)

        # Create and publish state message
        state_msg = Imu()
        state_msg.header.stamp = current_time.to_msg()
        state_msg.header.frame_id = self.imu_data.header.frame_id

        # For state estimation, we can use the same message format but interpret it differently
        state_msg.orientation.x = float(self.position[0])
        state_msg.orientation.y = float(self.position[1])
        state_msg.orientation.z = float(self.position[2])
        state_msg.orientation.w = 1.0  # Using w to indicate valid position

        state_msg.angular_velocity.x = float(self.velocity[0])
        state_msg.angular_velocity.y = float(self.velocity[1])
        state_msg.angular_velocity.z = float(self.velocity[2])

        self.state_pub.publish(state_msg)

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

    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Sensor Fusion Node interrupted by user')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()