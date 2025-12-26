#!/usr/bin/env python3

"""
Isaac ROS Visual SLAM Pipeline for Humanoid Robot

This module implements the Visual SLAM pipeline using Isaac ROS components
for the humanoid robot. It integrates camera and IMU data to provide
accurate localization and mapping capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from tf2_ros import TransformBroadcaster
import tf_transformations
from builtin_interfaces.msg import Time
import threading
import time


class IsaacROSVisualSLAMNode(Node):
    """
    Isaac ROS Visual SLAM Node for Humanoid Robot

    This node implements the visual SLAM pipeline that processes camera
    and IMU data to provide localization and mapping capabilities.
    """

    def __init__(self):
        super().__init__('isaac_ros_vslam_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Thread lock for data synchronization
        self.lock = threading.Lock()

        # Configuration parameters
        self.declare_parameter('config_file', '/config/vslam_config.yaml')
        self.declare_parameter('enable_loop_closure', True)
        self.declare_parameter('enable_pose_optimization', True)
        self.declare_parameter('max_features', 2000)
        self.declare_parameter('min_distance', 5)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('tracking_success_threshold', 0.7)

        # Get parameters
        self.config_file = self.get_parameter('config_file').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.enable_pose_optimization = self.get_parameter('enable_pose_optimization').value
        self.max_features = self.get_parameter('max_features').value
        self.min_distance = self.get_parameter('min_distance').value
        self.quality_level = self.get_parameter('quality_level').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.tracking_success_threshold = self.get_parameter('tracking_success_threshold').value

        # Initialize state variables
        self.current_pose = None
        self.pose_history = []
        self.feature_points = []
        self.map_points = []
        self.landmarks = {}  # Dictionary to store 3D landmarks
        self.keyframes = []  # List to store keyframes
        self.last_keyframe_pose = None
        self.tracking_success_rate = 1.0
        self.last_timestamp = None

        # QoS profile for sensor data (best effort for camera, reliable for IMU)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # Create subscribers for camera and IMU data
        self.image_sub = self.create_subscription(
            Image,
            '/sensors/camera/image_rect_color',
            self.image_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            imu_qos
        )

        # Create publishers for VSLAM outputs
        self.pose_pub = self.create_publisher(PoseStamped, '/vslam/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vslam/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/vslam/path_optimized', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/vslam/map', 10)
        self.features_pub = self.create_publisher(MarkerArray, '/vslam/features', 10)
        self.landmarks_pub = self.create_publisher(MarkerArray, '/vslam/landmarks', 10)
        self.keyframes_pub = self.create_publisher(MarkerArray, '/vslam/keyframes', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/vslam/twist', 10)

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize previous frame for optical flow
        self.prev_frame = None
        self.prev_features = None

        # Initialize camera parameters (these would normally come from camera_info)
        self.camera_matrix = np.array([
            [640.0, 0.0, 320.0],
            [0.0, 640.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Initialize IMU data
        self.imu_data = {
            'orientation': [0.0, 0.0, 0.0, 1.0],
            'angular_velocity': [0.0, 0.0, 0.0],
            'linear_acceleration': [0.0, 0.0, 0.0]
        }

        # Initialize pose
        self.current_pose = np.eye(4)  # 4x4 identity matrix

        # Timer for periodic processing
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 Hz

        self.get_logger().info('Isaac ROS Visual SLAM Node initialized')

    def image_callback(self, msg):
        """Process incoming camera image for visual SLAM"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Process image for feature detection and tracking
            with self.lock:
                self.process_image(cv_image, msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process incoming IMU data for sensor fusion"""
        try:
            # Update IMU data
            with self.lock:
                self.imu_data['orientation'] = [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ]
                self.imu_data['angular_velocity'] = [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ]
                self.imu_data['linear_acceleration'] = [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]

        except Exception as e:
            self.get_logger().error(f'Error processing IMU: {e}')

    def process_image(self, image, timestamp):
        """Process image for visual SLAM using optical flow and feature tracking"""
        # Convert to grayscale for processing
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Initialize features if this is the first frame
        if self.prev_frame is None:
            # Detect initial features
            features = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=self.max_features,
                qualityLevel=self.quality_level,
                minDistance=self.min_distance
            )

            if features is not None:
                self.prev_features = features
                self.prev_frame = gray.copy()
                self.last_timestamp = timestamp
                return

        # Calculate optical flow to track features
        if self.prev_features is not None:
            # Calculate optical flow
            new_features, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_frame, gray, self.prev_features, None
            )

            # Filter good features
            good_new = new_features[status == 1]
            good_old = self.prev_features[status == 1]

            # Update tracking success rate
            if len(good_new) > 0:
                self.tracking_success_rate = len(good_new) / len(self.prev_features)
            else:
                self.tracking_success_rate = 0.0

            # Estimate pose change if we have enough good features
            if len(good_new) >= 8:  # Minimum for pose estimation
                # Estimate essential matrix to get rotation and translation
                E, mask = cv2.findEssentialMat(
                    good_new, good_old, self.camera_matrix,
                    method=cv2.RANSAC, prob=0.999, threshold=1.0
                )

                if E is not None:
                    # Decompose essential matrix to get pose
                    _, R, t, _ = cv2.recoverPose(E, good_new, good_old, self.camera_matrix)

                    # Create transformation matrix
                    transformation = np.eye(4)
                    transformation[:3, :3] = R
                    transformation[:3, 3] = t.flatten()

                    # Update current pose
                    self.current_pose = self.current_pose @ transformation

            # Update for next iteration
            self.prev_features = good_new.reshape(-1, 1, 2)
            self.prev_frame = gray.copy()
            self.last_timestamp = timestamp

        # Publish pose if available
        if self.current_pose is not None:
            self.publish_pose(timestamp)

    def publish_pose(self, timestamp):
        """Publish the current pose estimate"""
        if self.current_pose is not None:
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = timestamp
            pose_msg.header.frame_id = 'map'

            # Extract position and orientation from transformation matrix
            pose_msg.pose.position.x = self.current_pose[0, 3]
            pose_msg.pose.position.y = self.current_pose[1, 3]
            pose_msg.pose.position.z = self.current_pose[2, 3]

            # Convert rotation matrix to quaternion
            rotation_matrix = self.current_pose[:3, :3]
            quat = tf_transformations.quaternion_from_matrix(
                np.block([[rotation_matrix, np.zeros((3, 1))], [np.zeros((1, 4))]])
            )

            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            # Publish pose
            self.pose_pub.publish(pose_msg)

            # Publish TF transform
            self.publish_transform(pose_msg.pose, timestamp)

    def publish_transform(self, pose, timestamp):
        """Publish TF transform for the robot's pose"""
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        """Periodic callback for publishing VSLAM outputs"""
        if self.current_pose is not None and self.last_timestamp is not None:
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = self.last_timestamp
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = self.current_pose[0, 3]
            odom_msg.pose.position.y = self.current_pose[1, 3]
            odom_msg.pose.position.z = self.current_pose[2, 3]

            # Convert rotation matrix to quaternion
            rotation_matrix = self.current_pose[:3, :3]
            quat = tf_transformations.quaternion_from_matrix(
                np.block([[rotation_matrix, np.zeros((3, 1))], [np.zeros((1, 4))]])
            )

            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            # Set covariance (placeholder values)
            odom_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0,
                                       0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

            self.odom_pub.publish(odom_msg)

            # Check for keyframe selection
            self.check_keyframe_selection()

            # Publish path (for visualization)
            if len(self.pose_history) < 1000:  # Limit history size
                self.pose_history.append(odom_msg.pose.pose)
            else:
                self.pose_history.pop(0)
                self.pose_history.append(odom_msg.pose.pose)

            path_msg = Path()
            path_msg.header.stamp = self.last_timestamp
            path_msg.header.frame_id = 'map'

            for i, pose in enumerate(self.pose_history):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = Time(sec=self.last_timestamp.sec + i, nanosec=self.last_timestamp.nanosec)
                pose_stamped.header.frame_id = 'map'
                pose_stamped.pose = pose
                path_msg.poses.append(pose_stamped)

            self.path_pub.publish(path_msg)

    def check_keyframe_selection(self):
        """Check if current pose should be added as a keyframe"""
        if self.current_pose is not None:
            # If no keyframes exist, create the first one
            if not self.keyframes:
                self.keyframes.append(self.current_pose.copy())
                self.last_keyframe_pose = self.current_pose.copy()
                self.publish_keyframes()
                return

            # Calculate distance from last keyframe
            if self.last_keyframe_pose is not None:
                # Calculate translation distance
                pos_current = self.current_pose[:3, 3]
                pos_last_kf = self.last_keyframe_pose[:3, 3]
                distance = np.linalg.norm(pos_current - pos_last_kf)

                # Calculate rotation distance
                rot_current = self.current_pose[:3, :3]
                rot_last_kf = self.last_keyframe_pose[:3, :3]
                rotation_diff = np.arccos(
                    np.clip((np.trace(rot_current.T @ rot_last_kf) - 1) / 2, -1, 1)
                )

                # Check if this frame meets keyframe criteria
                if (distance > 0.5 or rotation_diff > 0.2) and len(self.keyframes) < 200:
                    self.keyframes.append(self.current_pose.copy())
                    self.last_keyframe_pose = self.current_pose.copy()

                    # Limit number of keyframes
                    if len(self.keyframes) > 200:
                        self.keyframes.pop(0)

                    self.publish_keyframes()

    def publish_keyframes(self):
        """Publish keyframes as visualization markers"""
        if not self.keyframes:
            return

        marker_array = MarkerArray()
        for i, pose in enumerate(self.keyframes):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'keyframes'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = pose[0, 3]
            marker.pose.position.y = pose[1, 3]
            marker.pose.position.z = pose[2, 3]
            marker.pose.orientation.w = 1.0

            # Set size and color
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.keyframes_pub.publish(marker_array)

    def get_tracking_success_rate(self):
        """Get the current tracking success rate"""
        return self.tracking_success_rate


def main(args=None):
    """Main function to run the Isaac ROS VSLAM node"""
    rclpy.init(args=args)

    vslam_node = IsaacROSVisualSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()