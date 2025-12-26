#!/usr/bin/env python3
"""
Isaac ROS Object Detection Pipeline

This node implements an object detection pipeline using Isaac ROS components
for detecting and classifying objects in camera images.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class ObjectDetectionPipeline(Node):
    def __init__(self):
        super().__init__('object_detection_pipeline')

        # Declare parameters
        self.declare_parameter('input_image_topic', '/sensors/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/camera_info')
        self.declare_parameter('output_detections_topic', '/perception/objects')
        self.declare_parameter('model_name', 'detectnet_coco')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)
        self.declare_parameter('enable_preprocessing', True)
        self.declare_parameter('enable_postprocessing', True)
        self.declare_parameter('processing_frequency', 15.0)

        # Get parameters
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_detections_topic = self.get_parameter('output_detections_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.enable_preprocessing = self.get_parameter('enable_preprocessing').value
        self.enable_postprocessing = self.get_parameter('enable_postprocessing').value
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

        # Image processing variables
        self.latest_image = None
        self.image_lock = threading.Lock()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        # Create publisher
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            self.output_detections_topic,
            sensor_qos
        )

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.processing_frequency, self.run_detection)

        self.get_logger().info('Object Detection Pipeline initialized')
        self.get_logger().info(f'Input topic: {self.input_image_topic}')
        self.get_logger().info(f'Output topic: {self.output_detections_topic}')
        self.get_logger().info(f'Model: {self.model_name}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')

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

    def run_detection(self):
        """Run object detection on the latest image"""
        if self.latest_image is None:
            return

        with self.image_lock:
            image = self.latest_image.copy()

        # Preprocess image if enabled
        if self.enable_preprocessing:
            image = self.preprocess_image(image)

        # Run object detection (simulated)
        detections = self.detect_objects(image)

        # Postprocess detections if enabled
        if self.enable_postprocessing:
            detections = self.postprocess_detections(detections)

        # Publish detections
        detection_array_msg = self.create_detection_array_message(detections)
        self.detections_pub.publish(detection_array_msg)

        self.get_logger().debug(f'Published {len(detections)} detections')

    def preprocess_image(self, image):
        """Preprocess image for object detection"""
        # Resize image to model input size
        preprocessed = cv2.resize(image, (self.input_width, self.input_height))

        # Apply any necessary preprocessing (e.g., normalization)
        # For Isaac ROS DetectNet, the model typically expects BGR format
        return preprocessed

    def detect_objects(self, image):
        """Simulate object detection (in a real implementation, this would use Isaac ROS DetectNet)"""
        # In a real Isaac ROS implementation, this would interface with the DetectNet model
        # For this simulation, we'll use OpenCV's DNN module with a pre-trained model
        # or create simulated detections

        height, width = image.shape[:2]

        # Simulate detection results
        # In a real implementation, this would come from the DetectNet model
        simulated_detections = []

        # Create some simulated detections for testing
        # These would be actual detections from the model in a real implementation
        for i in range(np.random.randint(1, 4)):  # 1-3 random detections
            # Random bounding box
            x = np.random.randint(0, width // 2)
            y = np.random.randint(0, height // 2)
            w = np.random.randint(width // 4, width // 2)
            h = np.random.randint(height // 4, height // 2)

            # Ensure bounding box is within image bounds
            x = min(x, width - 1)
            y = min(y, height - 1)
            w = min(w, width - x)
            h = min(h, height - y)

            # Random confidence score
            confidence = np.random.uniform(self.confidence_threshold, 0.95)

            # Random class (for simulation)
            classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light']
            class_name = np.random.choice(classes)

            detection = {
                'class_name': class_name,
                'confidence': confidence,
                'bbox': {
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h
                }
            }

            simulated_detections.append(detection)

        return simulated_detections

    def postprocess_detections(self, detections):
        """Postprocess detections (e.g., apply NMS)"""
        if not detections:
            return detections

        # Apply Non-Maximum Suppression (NMS) to remove overlapping detections
        # This is a simplified implementation - in a real system, this would be handled by Isaac ROS
        boxes = []
        scores = []
        for det in detections:
            bbox = det['bbox']
            boxes.append([bbox['x'], bbox['y'], bbox['x'] + bbox['width'], bbox['y'] + bbox['height']])
            scores.append(det['confidence'])

        # Convert to numpy arrays
        boxes = np.array(boxes)
        scores = np.array(scores)

        # Apply NMS using OpenCV
        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(),
            scores.tolist(),
            self.confidence_threshold,
            self.nms_threshold
        )

        # Filter detections based on NMS results
        if len(indices) > 0:
            filtered_detections = [detections[i] for i in indices.flatten()]
        else:
            filtered_detections = []

        return filtered_detections

    def create_detection_array_message(self, detections):
        """Create Detection2DArray message from detection results"""
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()

        # Set frame_id from camera info or default
        if self.latest_image_msg:
            detection_array.header.frame_id = self.latest_image_msg.header.frame_id
        else:
            detection_array.header.frame_id = 'camera_rgb_optical_frame'

        for det in detections:
            detection_2d = Detection2D()
            detection_2d.header.stamp = detection_array.header.stamp
            detection_2d.header.frame_id = detection_array.header.frame_id

            # Set bounding box
            bbox = BoundingBox2D()
            bbox.center.x = float(det['bbox']['x'] + det['bbox']['width'] / 2.0)
            bbox.center.y = float(det['bbox']['y'] + det['bbox']['height'] / 2.0)
            bbox.center.z = 0.0
            bbox.size_x = float(det['bbox']['width'])
            bbox.size_y = float(det['bbox']['height'])
            detection_2d.bbox = bbox

            # Set detection results (class and confidence)
            from vision_msgs.msg import ObjectHypothesisWithPose
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = det['class_name']
            hypothesis.score = det['confidence']
            detection_2d.results = [hypothesis]

            detection_array.detections.append(detection_2d)

        return detection_array

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    detection_node = ObjectDetectionPipeline()

    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        detection_node.get_logger().info('Object Detection Pipeline interrupted by user')
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()