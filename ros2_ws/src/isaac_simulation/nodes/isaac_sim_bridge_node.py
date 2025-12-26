#!/usr/bin/env python3
"""
Isaac Sim Bridge Node

This node serves as a bridge between Isaac Sim and ROS 2, facilitating
the exchange of sensor data, robot state, and control commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, JointState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from tf2_msgs.msg import TFMessage, TransformStamped
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from datetime import datetime


class IsaacSimBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('publish_frequency', 60.0)
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('joint_states_topic', '/isaac_sim/joint_states')
        self.declare_parameter('tf_topic', '/isaac_sim/tf')

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.tf_topic = self.get_parameter('tf_topic').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Create QoS profile for state data
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publishers
        self.camera_pub = self.create_publisher(Image, self.camera_topic, sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, f'{self.camera_topic}/camera_info', sensor_qos)
        self.lidar_pub = self.create_publisher(PointCloud2, self.lidar_topic, sensor_qos)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, sensor_qos)
        self.joint_states_pub = self.create_publisher(JointState, self.joint_states_topic, state_qos)
        self.tf_pub = self.create_publisher(TFMessage, self.tf_topic, state_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', state_qos)

        # Create subscribers for control commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            state_qos
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            state_qos
        )

        # Robot state variables
        self.current_time = self.get_clock().now()
        self.previous_time = self.get_clock().now()
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.robot_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'torso_joint': 0.0,
            'head_joint': 0.0
        }

        # Simulation control
        self.simulation_running = True
        self.control_lock = threading.Lock()

        # Create timer for publishing sensor data
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_sensor_data)

        self.get_logger().info('Isaac Sim Bridge Node initialized')
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Publish frequency: {self.publish_frequency} Hz')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS"""
        with self.control_lock:
            # Convert Twist command to robot motion
            # This would interface with Isaac Sim's control system
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            # Update robot velocity based on command
            self.robot_velocity[0] = linear_x
            self.robot_velocity[1] = linear_y
            self.robot_angular_velocity[2] = angular_z

            self.get_logger().debug(f'Received cmd_vel: linear=({linear_x}, {linear_y}), angular=({angular_z})')

    def goal_pose_callback(self, msg):
        """Handle goal pose commands from ROS"""
        with self.control_lock:
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_z = msg.pose.position.z

            self.get_logger().info(f'Received goal pose: ({goal_x}, {goal_y}, {goal_z})')
            # This would trigger navigation in Isaac Sim

    def publish_sensor_data(self):
        """Publish sensor data from Isaac Sim to ROS topics"""
        self.current_time = self.get_clock().now()

        # Publish joint states
        self.publish_joint_states()

        # Publish TF transforms
        self.publish_tf()

        # Publish odometry
        self.publish_odometry()

        # Publish IMU data
        self.publish_imu_data()

        # In a real implementation, this would interface with Isaac Sim
        # to get actual sensor data. For now, we'll publish simulated data.

        # Publish simulated camera data
        self.publish_camera_data()

        # Publish simulated LiDAR data
        self.publish_lidar_data()

        # Update robot position based on velocity (simple integration)
        dt = (self.current_time.nanoseconds - self.previous_time.nanoseconds) / 1e9
        if dt > 0:
            self.robot_position += self.robot_velocity * dt
            # Update orientation based on angular velocity
            # Simplified: just update yaw for now
            yaw_change = self.robot_angular_velocity[2] * dt
            # Update orientation quaternion based on yaw change
            # This is a simplified approach - in reality, you'd use proper quaternion math
            self.get_logger().debug(f'Robot position: {self.robot_position}')

        self.previous_time = self.current_time

    def publish_joint_states(self):
        """Publish joint state data"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.current_time.to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        # Add joint names and positions
        joint_state_msg.name = list(self.joint_positions.keys())
        joint_state_msg.position = list(self.joint_positions.values())

        # For simplicity, set zero velocity and effort
        joint_state_msg.velocity = [0.0] * len(self.joint_positions)
        joint_state_msg.effort = [0.0] * len(self.joint_positions)

        self.joint_states_pub.publish(joint_state_msg)

    def publish_tf(self):
        """Publish transform data"""
        tf_msg = TFMessage()

        # Create transform from odom to base_link
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = self.current_time.to_msg()
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        odom_to_base.transform.translation.x = float(self.robot_position[0])
        odom_to_base.transform.translation.y = float(self.robot_position[1])
        odom_to_base.transform.translation.z = float(self.robot_position[2])
        odom_to_base.transform.rotation.x = 0.0  # Simplified
        odom_to_base.transform.rotation.y = 0.0
        odom_to_base.transform.rotation.z = 0.0
        odom_to_base.transform.rotation.w = 1.0

        tf_msg.transforms.append(odom_to_base)

        # Add other transforms (simplified)
        # In a real implementation, these would come from Isaac Sim
        for joint_name in self.joint_positions.keys():
            if 'hip' in joint_name or 'knee' in joint_name or 'ankle' in joint_name:
                link_name = joint_name.replace('_joint', '_link')
                joint_tf = TransformStamped()
                joint_tf.header.stamp = self.current_time.to_msg()
                joint_tf.header.frame_id = 'base_link'
                joint_tf.child_frame_id = link_name
                joint_tf.transform.translation.x = 0.0
                joint_tf.transform.translation.y = 0.0
                joint_tf.transform.translation.z = 0.0
                joint_tf.transform.rotation.x = 0.0
                joint_tf.transform.rotation.y = 0.0
                joint_tf.transform.rotation.z = 0.0
                joint_tf.transform.rotation.w = 1.0
                tf_msg.transforms.append(joint_tf)

        self.tf_pub.publish(tf_msg)

    def publish_odometry(self):
        """Publish odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = float(self.robot_position[0])
        odom_msg.pose.pose.position.y = float(self.robot_position[1])
        odom_msg.pose.pose.position.z = float(self.robot_position[2])

        # Set orientation (simplified)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Set velocity
        odom_msg.twist.twist.linear.x = float(self.robot_velocity[0])
        odom_msg.twist.twist.linear.y = float(self.robot_velocity[1])
        odom_msg.twist.twist.linear.z = float(self.robot_velocity[2])
        odom_msg.twist.twist.angular.x = float(self.robot_angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(self.robot_angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(self.robot_angular_velocity[2])

        self.odom_pub.publish(odom_msg)

    def publish_imu_data(self):
        """Publish IMU sensor data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation (simplified)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        # Set angular velocity
        imu_msg.angular_velocity.x = float(self.robot_angular_velocity[0])
        imu_msg.angular_velocity.y = float(self.robot_angular_velocity[1])
        imu_msg.angular_velocity.z = float(self.robot_angular_velocity[2])

        # Set linear acceleration (simplified - just gravity)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = -9.81

        # Set covariance matrices (set to zero for now)
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_pub.publish(imu_msg)

    def publish_camera_data(self):
        """Publish simulated camera data"""
        # Create a simple simulated image
        width = 640
        height = 480
        # Create a simple gradient image for simulation
        image_data = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                image_data[y, x, 0] = int((x / width) * 255)  # Red gradient
                image_data[y, x, 1] = int((y / height) * 255)  # Green gradient
                image_data[y, x, 2] = 100  # Blue constant

        # Create Image message
        img_msg = Image()
        img_msg.header.stamp = self.current_time.to_msg()
        img_msg.header.frame_id = 'camera_rgb_optical_frame'
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = width * 3  # 3 bytes per pixel for RGB
        img_msg.data = image_data.tobytes()

        self.camera_pub.publish(img_msg)

        # Publish camera info
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.current_time.to_msg()
        camera_info_msg.header.frame_id = 'camera_rgb_optical_frame'
        camera_info_msg.height = height
        camera_info_msg.width = width
        camera_info_msg.distortion_model = 'plumb_bob'
        camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        camera_info_msg.k = [320.0, 0.0, 320.0,  # fx, 0, cx
                             0.0, 320.0, 240.0,  # 0, fy, cy
                             0.0, 0.0, 1.0]      # 0, 0, 1
        camera_info_msg.r = [1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0]
        camera_info_msg.p = [320.0, 0.0, 320.0, 0.0,   # fx, 0, cx, 0
                             0.0, 320.0, 240.0, 0.0,   # 0, fy, cy, 0
                             0.0, 0.0, 1.0, 0.0]       # 0, 0, 1, 0

        self.camera_info_pub.publish(camera_info_msg)

    def publish_lidar_data(self):
        """Publish simulated LiDAR data"""
        # Create simulated point cloud data
        # For simplicity, create a basic point cloud with some points
        num_points = 100
        points = []
        for i in range(num_points):
            # Create points in a circular pattern around the robot
            angle = 2 * np.pi * i / num_points
            distance = 5.0 + np.random.uniform(-0.5, 0.5)  # 5m radius with noise
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            z = np.random.uniform(-0.5, 0.5)  # Small z variation
            points.extend([x, y, z])

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.current_time.to_msg()
        pc_msg.header.frame_id = 'velodyne'
        pc_msg.height = 1
        pc_msg.width = len(points) // 3
        pc_msg.fields = [
            {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},  # FLOAT32
            {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
            {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1}
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12  # 3 * 4 bytes (FLOAT32)
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.is_dense = True
        # Convert points to bytes (this is a simplified approach)
        # In a real implementation, you'd properly pack the data
        pc_msg.data = b''.join([val.to_bytes(4, 'little', signed=False) for val in points])

        self.lidar_pub.publish(pc_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.simulation_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    bridge_node = IsaacSimBridgeNode()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Isaac Sim Bridge interrupted by user')
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()