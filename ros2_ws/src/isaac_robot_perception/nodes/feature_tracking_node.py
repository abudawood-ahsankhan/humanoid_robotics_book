#!/usr/bin/env python3
"""
Isaac ROS Feature Tracking Node

This node implements feature detection, tracking, and mapping using Isaac ROS components
for Visual SLAM applications.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R


class FeatureTrackingNode(Node):
    def __init__(self):
        super().__init__('feature_tracking_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/camera_info')
        self.declare_parameter('features_topic', '/perception/features')
        self.declare_parameter('map_points_topic', '/perception/map_points')
        self.declare_parameter('keyframes_topic', '/perception/keyframes')
        self.declare_parameter('detector', 'orb')
        self.declare_parameter('max_features', 1000)
        self.declare_parameter('min_distance', 10)
        self.declare_parameter('processing_frequency', 30.0)

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.features_topic = self.get_parameter('features_topic').value
        self.map_points_topic = self.get_parameter('map_points_topic').value
        self.keyframes_topic = self.get_parameter('keyframes_topic').value
        self.detector_type = self.get_parameter('detector').value
        self.max_features = self.get_parameter('max_features').value
        self.min_distance = self.get_parameter('min_distance').value
        self.processing_frequency = self.get_parameter('processing_frequency').value

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

        # Initialize feature tracking variables
        self.latest_image = None
        self.image_lock = threading.Lock()

        # Feature tracking state
        self.current_features = None
        self.previous_features = None
        self.feature_ids = None
        self.next_feature_id = 0
        self.feature_tracks = {}  # Track features over time
        self.map_points = {}     # 3D map points
        self.keyframes = []      # Keyframes for mapping
        self.frame_count = 0

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
        self.features_pub = self.create_publisher(
            String,  # For simplicity, using String to publish feature info
            self.features_topic,
            sensor_qos
        )

        self.map_points_pub = self.create_publisher(
            String,  # For simplicity, using String to publish map point info
            self.map_points_topic,
            sensor_qos
        )

        self.keyframes_pub = self.create_publisher(
            String,  # For simplicity, using String to publish keyframe info
            self.keyframes_topic,
            sensor_qos
        )

        # Create timer for feature tracking
        self.feature_timer = self.create_timer(1.0 / self.processing_frequency, self.track_features)

        self.get_logger().info('Feature Tracking Node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Detector: {self.detector_type}')
        self.get_logger().info(f'Max features: {self.max_features}')
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

    def track_features(self):
        """Track features across frames"""
        if self.latest_image is None or not self.camera_info_received:
            return

        with self.image_lock:
            current_image = self.latest_image.copy()

        # Convert to grayscale for feature detection
        gray_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)

        # Detect features in current image
        current_keypoints, current_descriptors = self.detect_features(gray_image)

        # Track features if we have previous features
        if self.previous_features is not None and len(self.previous_features) > 0:
            # Track features using optical flow
            tracked_points, status, error = cv2.calcOpticalFlowPyrLK(
                cv2.cvtColor(self.previous_image, cv2.COLOR_BGR2GRAY),
                gray_image,
                self.previous_features,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
            )

            # Filter valid tracked points
            valid_mask = status.ravel() == 1
            self.current_features = tracked_points[valid_mask]
            prev_features = self.previous_features[valid_mask]

            # Update feature tracks
            self.update_feature_tracks(prev_features, self.current_features)

        else:
            # Initialize features if this is the first frame
            if len(current_keypoints) > 0:
                self.current_features = np.array([kp.pt for kp in current_keypoints], dtype=np.float32)
                self.feature_ids = np.arange(len(self.current_features)) + self.next_feature_id
                self.next_feature_id += len(self.current_features)

        # Store current data for next iteration
        self.previous_image = gray_image.copy()
        self.previous_features = self.current_features.copy() if self.current_features is not None else None

        # Add keyframe if conditions are met
        if self.should_add_keyframe():
            self.add_keyframe(current_image)

        # Publish feature information
        self.publish_features()
        self.publish_map_points()
        self.publish_keyframes()

        # Update frame count
        self.frame_count += 1

        if self.current_features is not None:
            self.get_logger().debug(f'Feature tracking frame {self.frame_count}: {len(self.current_features)} features tracked')

    def detect_features(self, image):
        """Detect features in the image using the specified detector"""
        if self.detector_type.lower() == 'orb':
            detector = cv2.ORB_create(nfeatures=self.max_features)
        elif self.detector_type.lower() == 'sift':
            detector = cv2.SIFT_create(nfeatures=self.max_features)
        elif self.detector_type.lower() == 'fast':
            detector = cv2.FastFeatureDetector_create()
        else:
            detector = cv2.ORB_create(nfeatures=self.max_features)  # Default to ORB

        keypoints, descriptors = detector.detectAndCompute(image, None)

        # Apply minimum distance filtering
        if keypoints:
            keypoints = self.filter_keypoints_by_distance(keypoints)

        return keypoints, descriptors

    def filter_keypoints_by_distance(self, keypoints):
        """Filter keypoints based on minimum distance"""
        if len(keypoints) == 0:
            return keypoints

        # Convert keypoints to points
        points = np.array([kp.pt for kp in keypoints], dtype=np.float32)

        # Filter based on minimum distance
        filtered_points = []
        for point in points:
            too_close = False
            for existing_point in filtered_points:
                dist = np.linalg.norm(point - existing_point)
                if dist < self.min_distance:
                    too_close = True
                    break
            if not too_close:
                filtered_points.append(point)

        # Convert back to keypoints
        filtered_keypoints = []
        for point in filtered_points:
            kp = cv2.KeyPoint(x=point[0], y=point[1], size=1.0)
            filtered_keypoints.append(kp)

        return filtered_keypoints

    def update_feature_tracks(self, prev_features, curr_features):
        """Update feature tracks with current positions"""
        # This is a simplified version - in a real implementation, this would
        # maintain longer-term feature tracks and associate them with 3D points
        for i, (prev_pt, curr_pt) in enumerate(zip(prev_features, curr_features)):
            # In a real implementation, we would maintain feature IDs and tracks
            # For now, we'll just store the current position
            feature_id = i
            self.feature_tracks[feature_id] = {
                'current_position': curr_pt,
                'tracked': True,
                'age': self.feature_tracks.get(feature_id, {}).get('age', 0) + 1
            }

    def should_add_keyframe(self):
        """Determine if a new keyframe should be added"""
        # Add keyframe every N frames for demonstration
        # In a real implementation, this would be based on motion, coverage, etc.
        return self.frame_count % 30 == 0  # Add keyframe every 30 frames

    def add_keyframe(self, image):
        """Add a new keyframe to the map"""
        keyframe = {
            'frame_id': self.frame_count,
            'image': image.copy(),
            'features': self.current_features.copy() if self.current_features is not None else [],
            'timestamp': self.get_clock().now().to_msg()
        }
        self.keyframes.append(keyframe)

        # Limit number of keyframes to prevent memory issues
        if len(self.keyframes) > 100:  # Keep only last 100 keyframes
            self.keyframes = self.keyframes[-100:]

        self.get_logger().debug(f'Added keyframe {len(self.keyframes)} at frame {self.frame_count}')

    def publish_features(self):
        """Publish feature information"""
        if self.current_features is not None:
            feature_info = {
                'frame_id': self.frame_count,
                'num_features': len(self.current_features),
                'avg_x': np.mean(self.current_features[:, 0]) if len(self.current_features) > 0 else 0,
                'avg_y': np.mean(self.current_features[:, 1]) if len(self.current_features) > 0 else 0
            }
            # For simplicity, publish as string - in real implementation, use proper message type
            feature_msg = String()
            feature_msg.data = str(feature_info)
            self.features_pub.publish(feature_msg)

    def publish_map_points(self):
        """Publish map point information"""
        # For simplicity, publish as string - in real implementation, use PointCloud2 or similar
        map_info = {
            'num_map_points': len(self.map_points),
            'num_keyframes': len(self.keyframes)
        }
        map_msg = String()
        map_msg.data = str(map_info)
        self.map_points_pub.publish(map_msg)

    def publish_keyframes(self):
        """Publish keyframe information"""
        # For simplicity, publish as string - in real implementation, use proper message type
        keyframe_info = {
            'current_keyframe_count': len(self.keyframes),
            'latest_frame': self.frame_count
        }
        keyframe_msg = String()
        keyframe_msg.data = str(keyframe_info)
        self.keyframes_pub.publish(keyframe_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    feature_node = FeatureTrackingNode()

    try:
        rclpy.spin(feature_node)
    except KeyboardInterrupt:
        feature_node.get_logger().info('Feature Tracking Node interrupted by user')
    finally:
        feature_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()