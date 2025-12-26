#!/usr/bin/env python3
"""
Isaac ROS Semantic Segmentation Pipeline

This node implements a semantic segmentation pipeline using Isaac ROS components
for pixel-level classification of objects in camera images.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class SemanticSegmentationPipeline(Node):
    def __init__(self):
        super().__init__('semantic_segmentation_pipeline')

        # Declare parameters
        self.declare_parameter('input_image_topic', '/sensors/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/camera_info')
        self.declare_parameter('output_segmentation_topic', '/perception/segmentation')
        self.declare_parameter('model_name', 'segway_coco')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)
        self.declare_parameter('enable_preprocessing', True)
        self.declare_parameter('enable_postprocessing', True)
        self.declare_parameter('processing_frequency', 10.0)

        # Get parameters
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_segmentation_topic = self.get_parameter('output_segmentation_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
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
        self.segmentation_pub = self.create_publisher(
            Image,  # Segmentation masks are typically published as images
            self.output_segmentation_topic,
            sensor_qos
        )

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.processing_frequency, self.run_segmentation)

        # Define color palette for segmentation classes
        self.class_colors = self.generate_class_colors(256)  # 256 possible classes

        self.get_logger().info('Semantic Segmentation Pipeline initialized')
        self.get_logger().info(f'Input topic: {self.input_image_topic}')
        self.get_logger().info(f'Output topic: {self.output_segmentation_topic}')
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

    def run_segmentation(self):
        """Run semantic segmentation on the latest image"""
        if self.latest_image is None:
            return

        with self.image_lock:
            image = self.latest_image.copy()

        # Preprocess image if enabled
        if self.enable_preprocessing:
            image = self.preprocess_image(image)

        # Run semantic segmentation (simulated)
        segmentation_mask = self.segment_image(image)

        # Postprocess segmentation if enabled
        if self.enable_postprocessing:
            segmentation_mask = self.postprocess_segmentation(segmentation_mask)

        # Create color segmentation image
        color_segmentation = self.create_color_segmentation(segmentation_mask)

        # Publish segmentation result
        try:
            segmentation_msg = self.bridge.cv2_to_imgmsg(color_segmentation, encoding='bgr8')
            segmentation_msg.header.stamp = self.get_clock().now().to_msg()
            if self.latest_image_msg:
                segmentation_msg.header.frame_id = self.latest_image_msg.header.frame_id
            else:
                segmentation_msg.header.frame_id = 'camera_rgb_optical_frame'
            self.segmentation_pub.publish(segmentation_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing segmentation: {e}')

        self.get_logger().debug('Published segmentation result')

    def preprocess_image(self, image):
        """Preprocess image for semantic segmentation"""
        # Resize image to model input size
        preprocessed = cv2.resize(image, (self.input_width, self.input_height))

        # Apply any necessary preprocessing (e.g., normalization)
        # For Isaac ROS Segmentation models, the model typically expects BGR format
        return preprocessed

    def segment_image(self, image):
        """Simulate semantic segmentation (in a real implementation, this would use Isaac ROS Segmentation)"""
        # In a real Isaac ROS implementation, this would interface with the segmentation model
        # For this simulation, we'll create a mock segmentation result

        height, width = image.shape[:2]

        # Simulate segmentation result
        # In a real implementation, this would come from the segmentation model
        # Create a segmentation mask with random class assignments
        segmentation_mask = np.zeros((height, width), dtype=np.uint8)

        # Create some random regions for simulation
        for i in range(np.random.randint(3, 8)):  # 3-8 random regions
            # Random center and size for the region
            center_x = np.random.randint(width // 4, 3 * width // 4)
            center_y = np.random.randint(height // 4, 3 * height // 4)
            radius = np.random.randint(min(width, height) // 8, min(width, height) // 4)

            # Create a circular region
            y, x = np.ogrid[:height, :width]
            mask = (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2

            # Assign a random class to this region
            class_id = np.random.randint(1, 21)  # Classes 1-20
            segmentation_mask[mask] = class_id

        return segmentation_mask

    def postprocess_segmentation(self, segmentation_mask):
        """Postprocess segmentation result (e.g., apply morphological operations)"""
        # Apply morphological operations to clean up the segmentation
        kernel = np.ones((3, 3), np.uint8)

        # Apply opening to remove small noise
        cleaned_mask = cv2.morphologyEx(segmentation_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # Apply closing to fill small holes
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        return cleaned_mask

    def create_color_segmentation(self, segmentation_mask):
        """Create a color image from the segmentation mask"""
        # Get the color palette
        color_palette = self.class_colors

        # Create RGB segmentation image
        height, width = segmentation_mask.shape
        color_segmentation = np.zeros((height, width, 3), dtype=np.uint8)

        # Map each class ID to its corresponding color
        for class_id in np.unique(segmentation_mask):
            if class_id > 0:  # Skip background (class 0)
                mask = segmentation_mask == class_id
                color_segmentation[mask] = color_palette[class_id]

        return color_segmentation

    def generate_class_colors(self, num_classes):
        """Generate distinct colors for each class"""
        colors = []
        for i in range(num_classes):
            # Generate color based on class ID
            # Using a deterministic method to ensure consistency
            np.random.seed(i)  # Use class ID as seed for reproducible colors
            color = np.random.randint(0, 255, size=3).tolist()
            colors.append(color)

        return np.array(colors, dtype=np.uint8)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    segmentation_node = SemanticSegmentationPipeline()

    try:
        rclpy.spin(segmentation_node)
    except KeyboardInterrupt:
        segmentation_node.get_logger().info('Semantic Segmentation Pipeline interrupted by user')
    finally:
        segmentation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()