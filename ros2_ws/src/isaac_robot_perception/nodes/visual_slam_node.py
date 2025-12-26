#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM Node

This node implements Visual SLAM using Isaac ROS components
for mapping and localization of the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R


class VisualSLAMNode(Node):
    def __init__(self):
        super().__init__('visual_slam_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/camera_info')
        self.declare_parameter('pose_topic', '/vslam/pose')
        self.declare_parameter('map_topic', '/vslam/map')
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('enable_mapping', True)
        self.declare_parameter('enable_loop_closure', True)
        self.declare_parameter('processing_frequency', 10.0)
        self.declare_parameter('max_features', 1000)
        self.declare_parameter('min_distance', 10)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.enable_mapping = self.get_parameter('enable_mapping').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.processing_frequency = self.get_parameter('processing_frequency').value
        self.max_features = self.get_parameter('max_features').value
        self.min_distance = self.get_parameter('min_distance').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize camera calibration parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False

        # Initialize SLAM variables
        self.latest_image = None
        self.image_lock = threading.Lock()

        # SLAM state variables
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes = []
        self.map_points = []
        self.feature_points = []
        self.previous_features = None
        self.previous_image = None
        self.frame_count = 0
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w

        # Create subscribers
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        # Create publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            sensor_qos
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.map_topic,
            sensor_qos
        )

        self.tf_pub = self.create_publisher(
            TFMessage,
            '/tf',
            sensor_qos
        )

        # Create timer for SLAM processing
        self.slam_timer = self.create_timer(1.0 / self.processing_frequency, self.run_slam)

        self.get_logger().info('Visual SLAM Node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Pose topic: {self.pose_topic}')
        self.get_logger().info(f'Map topic: {self.map_topic}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')

    def camera_info_callback(self, msg):
        """Callback for camera info messages"""
        if not self.camera_info_received:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera calibration parameters received')

    def image_callback(self, msg):
        """Callback for image messages"""
        with self.image_lock:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_image = cv_image.copy()
                self.latest_image_msg = msg  # Keep reference to the original message
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')

    def run_slam(self):
        """Run Visual SLAM algorithm"""
        if self.latest_image is None or not self.camera_info_received:
            return

        with self.image_lock:
            current_image = self.latest_image.copy()

        # Convert to grayscale for feature detection
        gray_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)

        # Feature detection and tracking
        if self.previous_image is not None:
            # Detect features in current image
            current_features = self.detect_features(gray_image)

            # Track features between previous and current frames
            if self.previous_features is not None and len(self.previous_features) > 0:
                # Calculate optical flow to track features
                tracked_points, status, error = cv2.calcOpticalFlowPyrLK(
                    self.previous_image, gray_image,
                    self.previous_features, None
                )

                # Filter valid points
                valid_points = status.ravel() == 1
                prev_points = self.previous_features[valid_points]
                curr_points = tracked_points[valid_points]

                if len(prev_points) >= 10:  # Need minimum points for pose estimation
                    # Estimate motion using Essential matrix
                    E, mask = cv2.findEssentialMat(
                        curr_points, prev_points,
                        self.camera_matrix,
                        method=cv2.RANSAC,
                        prob=0.999,
                        threshold=1.0
                    )

                    if E is not None:
                        # Recover pose from Essential matrix
                        _, R, t, _ = cv2.recoverPose(
                            E, curr_points, prev_points, self.camera_matrix
                        )

                        # Update position and orientation
                        self.update_pose(R, t)

                        # Publish pose
                        self.publish_pose()

                        # Update TF
                        self.publish_transform()

                        # Add keyframe if significant motion
                        if self.should_add_keyframe(R, t):
                            self.add_keyframe(gray_image, self.position.copy(), self.orientation.copy())

        # Store current data for next iteration
        self.previous_image = gray_image
        self.previous_features = self.detect_features(gray_image)

        # Publish map (simulated)
        self.publish_map()

        # Update frame count
        self.frame_count += 1

        self.get_logger().debug(f'SLAM iteration {self.frame_count}, pose: {self.position}')

    def detect_features(self, image):
        """Detect features in the image"""
        # Use ORB for feature detection (in a real implementation, this would use Isaac ROS feature detection)
        orb = cv2.ORB_create(nfeatures=self.max_features)
        keypoints = orb.detect(image, None)

        if keypoints:
            # Extract keypoint locations
            features = np.array([[kp.pt[0], kp.pt[1]] for kp in keypoints], dtype=np.float32)
            # Filter features based on minimum distance
            features = self.filter_features_by_distance(features)
            return features
        else:
            return np.array([]).reshape(-1, 1, 2)

    def filter_features_by_distance(self, features):
        """Filter features based on minimum distance"""
        if len(features) == 0:
            return features

        # Simple distance-based filtering
        filtered_features = []
        for feature in features:
            too_close = False
            for existing_feature in filtered_features:
                dist = np.linalg.norm(feature - existing_feature)
                if dist < self.min_distance:
                    too_close = True
                    break
            if not too_close:
                filtered_features.append(feature)

        return np.array(filtered_features)

    def update_pose(self, R, t):
        """Update robot pose based on rotation and translation"""
        # Convert rotation matrix to quaternion
        r = R.from_matrix(R)
        quat = r.as_quat()  # Returns [x, y, z, w]

        # Update position (scale translation appropriately)
        translation = t.flatten() * 0.1  # Scale factor for simulation
        self.position += translation

        # Update orientation
        self.orientation = quat

    def should_add_keyframe(self, R, t):
        """Determine if a new keyframe should be added"""
        # Check if motion is significant enough to warrant a new keyframe
        rotation_angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        translation_norm = np.linalg.norm(t)

        # Add keyframe if rotation > 10 degrees or translation > 0.1m
        return rotation_angle > 0.17 or translation_norm > 0.1

    def add_keyframe(self, image, position, orientation):
        """Add a new keyframe to the map"""
        keyframe = {
            'image': image.copy(),
            'position': position,
            'orientation': orientation,
            'features': self.detect_features(image)
        }
        self.keyframes.append(keyframe)

        # Limit number of keyframes to prevent memory issues
        if len(self.keyframes) > 100:  # Keep only last 100 keyframes
            self.keyframes = self.keyframes[-100:]

    def publish_pose(self):
        """Publish current pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])

        pose_msg.pose.orientation.x = float(self.orientation[0])
        pose_msg.pose.orientation.y = float(self.orientation[1])
        pose_msg.pose.orientation.z = float(self.orientation[2])
        pose_msg.pose.orientation.w = float(self.orientation[3])

        self.pose_pub.publish(pose_msg)

    def publish_transform(self):
        """Publish transform between map and odom frames"""
        tf_msg = TFMessage()

        # Create transform from map to odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame

        transform.transform.translation.x = float(self.position[0])
        transform.transform.translation.y = float(self.position[1])
        transform.transform.translation.z = float(self.position[2])

        transform.transform.rotation.x = float(self.orientation[0])
        transform.transform.rotation.y = float(self.orientation[1])
        transform.transform.rotation.z = float(self.orientation[2])
        transform.transform.rotation.w = float(self.orientation[3])

        tf_msg.transforms.append(transform)

        # Create transform from odom to base_link
        base_transform = TransformStamped()
        base_transform.header.stamp = self.get_clock().now().to_msg()
        base_transform.header.frame_id = self.odom_frame
        base_transform.child_frame_id = self.base_frame

        # For simplicity, assume base_link is at origin of odom frame
        base_transform.transform.translation.x = 0.0
        base_transform.transform.translation.y = 0.0
        base_transform.transform.translation.z = 0.0
        base_transform.transform.rotation.x = 0.0
        base_transform.transform.rotation.y = 0.0
        base_transform.transform.rotation.z = 0.0
        base_transform.transform.rotation.w = 1.0

        tf_msg.transforms.append(base_transform)

        self.tf_pub.publish(tf_msg)

    def publish_map(self):
        """Publish occupancy grid map (simulated)"""
        # Create a simple simulated map
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.map_frame

        # Map parameters
        map_msg.info.resolution = 0.1  # 10cm resolution
        map_msg.info.width = 1000      # 100m x 100m map
        map_msg.info.height = 1000
        map_msg.info.origin.position.x = -50.0  # Map centered at robot start position
        map_msg.info.origin.position.y = -50.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Initialize map with unknown values (-1)
        map_data = [-1] * (map_msg.info.width * map_msg.info.height)
        map_msg.data = map_data

        # Publish the map
        self.map_pub.publish(map_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    slam_node = VisualSLAMNode()

    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        slam_node.get_logger().info('Visual SLAM Node interrupted by user')
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()