#!/usr/bin/env python3
"""
Isaac ROS Camera Processing Node

This node processes camera data using Isaac ROS components for
rectification, feature detection, object detection, and segmentation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray
import cv2
import numpy as np
import threading
import time


class CameraProcessingNode(Node):
    def __init__(self):
        super().__init__('camera_processing_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/camera_info')
        self.declare_parameter('rectified_image_topic', '/perception/camera/rectified_image')
        self.declare_parameter('processed_image_topic', '/perception/camera/processed_image')
        self.declare_parameter('detections_topic', '/perception/objects')
        self.declare_parameter('enable_rectification', True)
        self.declare_parameter('enable_enhancement', True)
        self.declare_parameter('enable_feature_detection', True)
        self.declare_parameter('enable_object_detection', True)
        self.declare_parameter('processing_frequency', 30.0)

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.rectified_image_topic = self.get_parameter('rectified_image_topic').value
        self.processed_image_topic = self.get_parameter('processed_image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.enable_rectification = self.get_parameter('enable_rectification').value
        self.enable_enhancement = self.get_parameter('enable_enhancement').value
        self.enable_feature_detection = self.get_parameter('enable_feature_detection').value
        self.enable_object_detection = self.get_parameter('enable_object_detection').value
        self.processing_frequency = self.get_parameter('processing_frequency').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Create CvBridge
        self.bridge = CvBridge()

        # Initialize camera calibration parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.new_camera_matrix = None
        self.roi = None
        self.map1 = None
        self.map2 = None
        self.calibration_ready = False

        # Image processing flags
        self.latest_image = None
        self.image_lock = threading.Lock()

        # Create subscribers
        self.image_sub = self.create_subscription(
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
        self.rectified_pub = self.create_publisher(
            Image,
            self.rectified_image_topic,
            sensor_qos
        )

        self.processed_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            sensor_qos
        )

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            self.detections_topic,
            sensor_qos
        )

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.processing_frequency, self.process_image)

        self.get_logger().info('Camera Processing Node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')

    def camera_info_callback(self, msg):
        """Callback for camera info messages"""
        if not self.calibration_ready:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)

            # Compute optimal camera matrix and ROI for rectification
            h, w = msg.height, msg.width
            self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix,
                self.distortion_coeffs,
                (w, h),
                1,
                (w, h)
            )

            # Precompute map for rectification
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.distortion_coeffs,
                None,
                self.new_camera_matrix,
                (w, h),
                cv2.CV_32FC1
            )

            self.calibration_ready = True
            self.get_logger().info('Camera calibration parameters loaded')

    def image_callback(self, msg):
        """Callback for image messages"""
        with self.image_lock:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_image = cv_image.copy()
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')

    def process_image(self):
        """Process the latest image with all enabled processing steps"""
        if self.latest_image is None or not self.calibration_ready:
            return

        with self.image_lock:
            image = self.latest_image.copy()

        # Step 1: Rectification
        if self.enable_rectification:
            image = self.rectify_image(image)

        # Store original for later use
        original_image = image.copy()

        # Step 2: Enhancement
        if self.enable_enhancement:
            image = self.enhance_image(image)

        # Step 3: Feature detection
        if self.enable_feature_detection:
            image_with_features = image.copy()
            keypoints = self.detect_features(image_with_features)
            # Draw keypoints on image
            for kp in keypoints:
                cv2.circle(image_with_features, tuple(map(int, kp.pt)), 3, (0, 255, 0), -1)

            # Publish image with features
            try:
                feature_image_msg = self.bridge.cv2_to_imgmsg(image_with_features, encoding='bgr8')
                feature_image_msg.header = self.latest_image_msg.header if hasattr(self, 'latest_image_msg') else image_with_features
                self.processed_pub.publish(feature_image_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing feature image: {e}')

        # Step 4: Object detection (simulated)
        if self.enable_object_detection:
            detections = self.simulate_object_detection(image)
            try:
                self.detections_pub.publish(detections)
            except Exception as e:
                self.get_logger().error(f'Error publishing detections: {e}')

        # Publish rectified image
        try:
            rectified_image_msg = self.bridge.cv2_to_imgmsg(original_image, encoding='bgr8')
            rectified_image_msg.header = self.latest_image_msg.header if hasattr(self, 'latest_image_msg') else original_image
            self.rectified_pub.publish(rectified_image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing rectified image: {e}')

        # Publish processed image (after enhancement)
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            processed_image_msg.header = self.latest_image_msg.header if hasattr(self, 'latest_image_msg') else image
            self.processed_pub.publish(processed_image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')

    def rectify_image(self, image):
        """Rectify the image using camera calibration parameters"""
        if self.map1 is not None and self.map2 is not None:
            # Apply the precomputed rectification map
            rectified = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)

            # Crop the image based on ROI
            x, y, w, h = self.roi
            rectified = rectified[y:y+h, x:x+w]

            return rectified
        else:
            return image

    def enhance_image(self, image):
        """Enhance the image using various techniques"""
        enhanced = image.copy()

        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        if len(enhanced.shape) == 3:
            # Convert to LAB color space for better enhancement
            lab = cv2.cvtColor(enhanced, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)

            # Apply CLAHE to the L channel
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            l = clahe.apply(l)

            # Merge the channels back
            enhanced = cv2.merge([l, a, b])
            enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        else:
            # For grayscale images
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced = clahe.apply(enhanced)

        # Apply bilateral filter for noise reduction while preserving edges
        enhanced = cv2.bilateralFilter(enhanced, 9, 75, 75)

        return enhanced

    def detect_features(self, image):
        """Detect features in the image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        # Use Shi-Tomasi corner detection as an example
        # In a real implementation, this would use Isaac ROS feature detection
        corners = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=1000,
            qualityLevel=0.01,
            minDistance=10
        )

        keypoints = []
        if corners is not None:
            for corner in corners:
                x, y = corner.ravel()
                keypoints.append(cv2.KeyPoint(x, y, size=3))

        return keypoints

    def simulate_object_detection(self, image):
        """Simulate object detection (in a real implementation, this would use Isaac ROS DetectNet)"""
        from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
        from geometry_msgs.msg import Point
        from std_msgs.msg import Header

        detections = Detection2DArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = 'camera_rgb_optical_frame'  # This should come from image header

        # Simulate some detections for testing purposes
        # In a real implementation, this would use Isaac ROS DetectNet
        height, width = image.shape[:2]

        # Create a simulated detection
        detection = Detection2D()
        detection.header.stamp = detections.header.stamp
        detection.header.frame_id = detections.header.frame_id

        # Bounding box (simulated)
        bbox = BoundingBox2D()
        bbox.size_x = 100.0
        bbox.size_y = 100.0
        center = Point()
        center.x = width / 2.0
        center.y = height / 2.0
        center.z = 0.0
        bbox.center = center
        detection.bbox = bbox

        # Detection results (simulated)
        # In a real implementation, this would come from the detection model
        from vision_msgs.msg import ObjectHypothesisWithPose
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = 'person'  # Simulated detection class
        hypothesis.score = 0.85   # Simulated confidence
        detection.results = [hypothesis]

        detections.detections = [detection]

        return detections

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraProcessingNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info('Camera Processing Node interrupted by user')
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()