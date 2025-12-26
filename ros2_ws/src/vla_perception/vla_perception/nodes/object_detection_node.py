#!/usr/bin/env python3

"""
Object Detection Node for VLA System

This node detects objects in images using computer vision techniques.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from vla_msgs.msg import ObjectList, ObjectQuery, Object
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create subscriber for object queries
        self.query_sub = self.create_subscription(
            ObjectQuery,
            '/vla/query_objects',
            self.query_callback,
            10
        )

        # Create publisher for detected objects
        self.objects_pub = self.create_publisher(ObjectList, '/vla/detected_objects', 10)

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Object detection parameters
        self.detection_threshold = 0.5
        self.enable_tracking = True

        self.get_logger().info('Object Detection Node initialized')

    def image_callback(self, msg):
        """Process incoming image and detect objects"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detected_objects = self.detect_objects(cv_image)

            # Create and publish object list
            object_list_msg = ObjectList()
            object_list_msg.objects = detected_objects
            self.objects_pub.publish(object_list_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def query_callback(self, msg):
        """Handle object query requests"""
        # Process the query and detect specific object types
        # For now, just do general detection
        pass

    def detect_objects(self, image):
        """Detect objects in the image using a simple method"""
        objects = []

        # This is a placeholder implementation
        # In a real system, we would use YOLO, Detectron2, or similar
        height, width = image.shape[:2]

        # Create some dummy objects for demonstration
        dummy_objects = [
            {
                'class': 'cup',
                'confidence': 0.85,
                'bbox': [width * 0.4, height * 0.4, width * 0.1, height * 0.1]
            },
            {
                'class': 'ball',
                'confidence': 0.78,
                'bbox': [width * 0.6, height * 0.5, width * 0.08, height * 0.08]
            }
        ]

        for i, obj in enumerate(dummy_objects):
            if obj['confidence'] > self.detection_threshold:
                # Create object message
                object_msg = Object()
                object_msg.object_id = f"obj_{i}"
                object_msg.class_name = obj['class']
                object_msg.confidence = obj['confidence']

                # Set pose (simplified - in real system, would use depth info)
                pose = Pose()
                pose.position.x = obj['bbox'][0] / width
                pose.position.y = obj['bbox'][1] / height
                pose.position.z = 1.0  # Default depth
                object_msg.pose = pose

                # Set dimensions (simplified)
                object_msg.dimensions.x = obj['bbox'][2] / width
                object_msg.dimensions.y = obj['bbox'][3] / height
                object_msg.dimensions.z = 0.1  # Default depth

                objects.append(object_msg)

        return objects


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()