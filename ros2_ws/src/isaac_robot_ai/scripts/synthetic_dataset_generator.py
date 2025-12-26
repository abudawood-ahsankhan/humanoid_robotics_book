#!/usr/bin/env python3
"""
Synthetic Dataset Generator for Isaac-based AI Training

This script generates synthetic datasets for AI training using Isaac Sim data.
It captures images, annotations, and sensor data from simulation to create
training datasets for perception and navigation tasks.
"""

import os
import json
import yaml
import cv2
import numpy as np
import argparse
from datetime import datetime
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class SyntheticDatasetGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_dataset_generator')

        # Declare parameters
        self.declare_parameter('output_dir', '/tmp/synthetic_dataset')
        self.declare_parameter('dataset_name', 'isaac_humanoid_dataset')
        self.declare_parameter('image_topic', '/sensors/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/camera_info')
        self.declare_parameter('detection_topic', '/perception/objects')
        self.declare_parameter('max_samples', 1000)
        self.declare_parameter('capture_frequency', 1.0)  # Hz

        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.dataset_name = self.get_parameter('dataset_name').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.max_samples = self.get_parameter('max_samples').value
        self.capture_frequency = self.get_parameter('capture_frequency').value

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize counters and data storage
        self.sample_count = 0
        self.image_buffer = None
        self.camera_info_buffer = None
        self.detection_buffer = None
        self.data_buffer = []

        # Create output directory
        self.dataset_dir = Path(self.output_dir) / self.dataset_name
        self.dataset_dir.mkdir(parents=True, exist_ok=True)

        # Create subdirectories
        (self.dataset_dir / 'images').mkdir(exist_ok=True)
        (self.dataset_dir / 'labels').mkdir(exist_ok=True)
        (self.dataset_dir / 'camera_info').mkdir(exist_ok=True)

        # Create ROS subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.detection_callback,
            10
        )

        # Create timer for data capture
        self.capture_timer = self.create_timer(1.0 / self.capture_frequency, self.capture_data)

        self.get_logger().info(f'Synthetic Dataset Generator initialized')
        self.get_logger().info(f'Output directory: {self.dataset_dir}')
        self.get_logger().info(f'Max samples: {self.max_samples}')
        self.get_logger().info(f'Capture frequency: {self.capture_frequency} Hz')

    def image_callback(self, msg):
        """Callback for image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_buffer = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def camera_info_callback(self, msg):
        """Callback for camera info messages"""
        self.camera_info_buffer = msg

    def detection_callback(self, msg):
        """Callback for detection messages"""
        self.detection_buffer = msg

    def capture_data(self):
        """Capture synchronized data from all sensors"""
        if self.sample_count >= self.max_samples:
            self.get_logger().info('Maximum sample count reached. Stopping data capture.')
            self.capture_timer.cancel()
            self.save_dataset_info()
            return

        if self.image_buffer is not None and self.camera_info_buffer is not None:
            # Create timestamp for this sample
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

            # Save image
            image_filename = f'{timestamp}.png'
            image_path = self.dataset_dir / 'images' / image_filename
            cv2.imwrite(str(image_path), self.image_buffer)

            # Save camera info
            camera_info_filename = f'{timestamp}_camera_info.json'
            camera_info_path = self.dataset_dir / 'camera_info' / camera_info_filename
            camera_info_data = {
                'header': {
                    'stamp': {
                        'sec': self.camera_info_buffer.header.stamp.sec,
                        'nanosec': self.camera_info_buffer.header.stamp.nanosec
                    },
                    'frame_id': self.camera_info_buffer.header.frame_id
                },
                'height': self.camera_info_buffer.height,
                'width': self.camera_info_buffer.width,
                'distortion_model': self.camera_info_buffer.distortion_model,
                'd': list(self.camera_info_buffer.d),
                'k': list(self.camera_info_buffer.k),
                'r': list(self.camera_info_buffer.r),
                'p': list(self.camera_info_buffer.p),
                'binning_x': self.camera_info_buffer.binning_x,
                'binning_y': self.camera_info_buffer.binning_y,
                'roi': {
                    'x_offset': self.camera_info_buffer.roi.x_offset,
                    'y_offset': self.camera_info_buffer.roi.y_offset,
                    'height': self.camera_info_buffer.roi.height,
                    'width': self.camera_info_buffer.roi.width,
                    'do_rectify': self.camera_info_buffer.roi.do_rectify
                }
            }

            with open(camera_info_path, 'w') as f:
                json.dump(camera_info_data, f, indent=2)

            # Process and save detections if available
            if self.detection_buffer is not None:
                detections_filename = f'{timestamp}_detections.json'
                detections_path = self.dataset_dir / 'labels' / detections_filename
                detections_data = self.process_detections(self.detection_buffer)

                with open(detections_path, 'w') as f:
                    json.dump(detections_data, f, indent=2)
            else:
                # Create empty detections file
                detections_filename = f'{timestamp}_detections.json'
                detections_path = self.dataset_dir / 'labels' / detections_filename
                with open(detections_path, 'w') as f:
                    json.dump({'detections': []}, f, indent=2)

            self.sample_count += 1
            self.get_logger().info(f'Captured sample {self.sample_count}/{self.max_samples}')

            # Clear buffers
            self.image_buffer = None
            self.camera_info_buffer = None
            self.detection_buffer = None

    def process_detections(self, detection_array_msg):
        """Process detection messages and convert to JSON format"""
        detections = []

        for detection in detection_array_msg.detections:
            detection_data = {
                'header': {
                    'stamp': {
                        'sec': detection.header.stamp.sec,
                        'nanosec': detection.header.stamp.nanosec
                    },
                    'frame_id': detection.header.frame_id
                },
                'id': detection.id,
                'results': [],
                'bbox': {
                    'size_x': detection.bbox.size_x,
                    'size_y': detection.bbox.size_y,
                    'center_x': detection.bbox.center.x,
                    'center_y': detection.bbox.center.y
                }
            }

            for result in detection.results:
                result_data = {
                    'id': result.id,
                    'score': result.score
                }
                detection_data['results'].append(result_data)

            detections.append(detection_data)

        return {'detections': detections}

    def save_dataset_info(self):
        """Save dataset information file"""
        dataset_info = {
            'name': self.dataset_name,
            'created': datetime.now().isoformat(),
            'total_samples': self.sample_count,
            'max_samples': self.max_samples,
            'capture_frequency': self.capture_frequency,
            'image_topic': self.image_topic,
            'camera_info_topic': self.camera_info_topic,
            'detection_topic': self.detection_topic,
            'directories': {
                'images': 'images/',
                'labels': 'labels/',
                'camera_info': 'camera_info/'
            },
            'file_formats': {
                'images': 'PNG',
                'labels': 'JSON',
                'camera_info': 'JSON'
            }
        }

        info_path = self.dataset_dir / 'dataset_info.json'
        with open(info_path, 'w') as f:
            json.dump(dataset_info, f, indent=2)

        self.get_logger().info(f'Dataset info saved to {info_path}')
        self.get_logger().info(f'Dataset generation completed with {self.sample_count} samples')


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Synthetic Dataset Generator for Isaac-based AI Training')
    parser.add_argument('--output_dir', type=str, default='/tmp/synthetic_dataset',
                       help='Output directory for dataset')
    parser.add_argument('--dataset_name', type=str, default='isaac_humanoid_dataset',
                       help='Name of the dataset')
    parser.add_argument('--max_samples', type=int, default=1000,
                       help='Maximum number of samples to capture')
    parser.add_argument('--capture_frequency', type=float, default=1.0,
                       help='Capture frequency in Hz')

    parsed_args = parser.parse_args()

    generator = SyntheticDatasetGenerator()

    # Override parameters with command line arguments
    generator.set_parameters([
        rclpy.parameter.Parameter('output_dir', rclpy.parameter.Parameter.Type.STRING, parsed_args.output_dir),
        rclpy.parameter.Parameter('dataset_name', rclpy.parameter.Parameter.Type.STRING, parsed_args.dataset_name),
        rclpy.parameter.Parameter('max_samples', rclpy.parameter.Parameter.Type.INTEGER, parsed_args.max_samples),
        rclpy.parameter.Parameter('capture_frequency', rclpy.parameter.Parameter.Type.DOUBLE, parsed_args.capture_frequency)
    ])

    try:
        rclpy.spin(generator)
    except KeyboardInterrupt:
        generator.get_logger().info('Dataset generation interrupted by user')
    finally:
        generator.save_dataset_info()
        generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()