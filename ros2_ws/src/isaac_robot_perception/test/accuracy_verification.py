#!/usr/bin/env python3
"""
Accuracy Verification for Isaac ROS Perception Pipeline

This script verifies the accuracy of the Isaac ROS perception pipeline
using ground truth data from Isaac Sim.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
import threading
import json
from pathlib import Path


class AccuracyVerificationNode(Node):
    def __init__(self):
        super().__init__('accuracy_verification_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('detections_topic', '/perception/objects')
        self.declare_parameter('ground_truth_topic', '/ground_truth/objects')
        self.declare_parameter('test_duration', 60.0)  # seconds
        self.declare_parameter('iou_threshold', 0.5)  # Intersection over Union threshold
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('position_tolerance', 0.1)  # meters
        self.declare_parameter('save_results', True)
        self.declare_parameter('results_directory', '/tmp/perception_accuracy_results')

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.ground_truth_topic = self.get_parameter('ground_truth_topic').value
        self.test_duration = self.get_parameter('test_duration').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.save_results = self.get_parameter('save_results').value
        self.results_directory = self.get_parameter('results_directory').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize test variables
        self.test_start_time = None
        self.test_active = False
        self.results = {
            'detection_metrics': [],
            'position_metrics': [],
            'overall_accuracy': 0.0,
            'precision': 0.0,
            'recall': 0.0,
            'f1_score': 0.0,
            'mAP': 0.0
        }

        # Data storage
        self.detections_buffer = deque(maxlen=100)
        self.ground_truth_buffer = deque(maxlen=100)
        self.image_buffer = deque(maxlen=10)
        self.data_lock = threading.Lock()

        # Create subscribers
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            sensor_qos
        )

        self.detections_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detections_callback,
            sensor_qos
        )

        self.ground_truth_sub = self.create_subscription(
            Detection2DArray,
            self.ground_truth_topic,
            self.ground_truth_callback,
            sensor_qos
        )

        self.get_logger().info('Accuracy Verification Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} seconds')
        self.get_logger().info(f'IOU threshold: {self.iou_threshold}')
        self.get_logger().info(f'Position tolerance: {self.position_tolerance} meters')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.image_buffer.append(msg)

    def detections_callback(self, msg):
        """Callback for detection messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.detections_buffer.append(msg)

    def ground_truth_callback(self, msg):
        """Callback for ground truth messages"""
        if not self.test_active:
            return

        with self.data_lock:
            self.ground_truth_buffer.append(msg)

    def start_test(self):
        """Start the accuracy verification test"""
        self.get_logger().info('Starting accuracy verification test...')
        self.test_start_time = self.get_clock().now()
        self.test_active = True

        # Wait for test duration
        start_time = time.time()
        while time.time() - start_time < self.test_duration and self.test_active:
            time.sleep(0.1)

        # Stop test
        self.test_active = False
        self.get_logger().info('Accuracy verification test completed')

        # Process results
        self.process_results()

        # Generate and print report
        self.generate_accuracy_report()

        # Save results if requested
        if self.save_results:
            self.save_test_results()

    def process_results(self):
        """Process the collected data to calculate accuracy metrics"""
        with self.data_lock:
            # Process detection accuracy
            detection_metrics = []
            position_metrics = []

            # Match detections with ground truth
            for det_msg, gt_msg in zip(self.detections_buffer, self.ground_truth_buffer):
                # Calculate detection accuracy metrics
                metrics = self.calculate_detection_metrics(det_msg, gt_msg)
                detection_metrics.append(metrics)

                # Calculate position accuracy metrics
                pos_metrics = self.calculate_position_metrics(det_msg, gt_msg)
                position_metrics.append(pos_metrics)

            # Calculate overall metrics
            if detection_metrics:
                # Calculate precision, recall, F1-score
                total_tp = sum(m['tp'] for m in detection_metrics)
                total_fp = sum(m['fp'] for m in detection_metrics)
                total_fn = sum(m['fn'] for m in detection_metrics)

                precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0
                recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0
                f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

                # Calculate mAP (mean Average Precision)
                ap_values = [m['ap'] for m in detection_metrics if m['ap'] is not None]
                mAP = np.mean(ap_values) if ap_values else 0

                self.results['detection_metrics'] = detection_metrics
                self.results['position_metrics'] = position_metrics
                self.results['precision'] = precision
                self.results['recall'] = recall
                self.results['f1_score'] = f1_score
                self.results['mAP'] = mAP
                self.results['overall_accuracy'] = (precision + recall) / 2

    def calculate_detection_metrics(self, detections, ground_truth):
        """Calculate detection accuracy metrics (TP, FP, FN, AP)"""
        # Convert detection arrays to comparable format
        det_objects = self.detections_to_objects(detections)
        gt_objects = self.detections_to_objects(ground_truth)

        # Initialize counters
        tp = 0  # True Positives
        fp = 0  # False Positives
        fn = 0  # False Negatives

        # Track matched ground truth objects
        matched_gt = set()

        # For each detection, find best matching ground truth
        for det in det_objects:
            best_match = None
            best_iou = 0

            # Find best matching ground truth object
            for i, gt in enumerate(gt_objects):
                if i in matched_gt:
                    continue  # Skip already matched ground truth

                iou = self.calculate_iou(det['bbox'], gt['bbox'])
                if iou > best_iou and iou >= self.iou_threshold:
                    best_iou = iou
                    best_match = i

            if best_match is not None:
                # Check if class matches
                if det['class'] == gt_objects[best_match]['class']:
                    tp += 1
                    matched_gt.add(best_match)
                else:
                    fp += 1
            else:
                fp += 1

        # Count false negatives (unmatched ground truth objects)
        fn = len(gt_objects) - len(matched_gt)

        # Calculate Average Precision (AP) - simplified approach
        if len(det_objects) > 0:
            ap = tp / len(det_objects)  # Simplified AP calculation
        else:
            ap = 0

        return {
            'tp': tp,
            'fp': fp,
            'fn': fn,
            'ap': ap,
            'total_detections': len(det_objects),
            'total_ground_truth': len(gt_objects)
        }

    def calculate_position_metrics(self, detections, ground_truth):
        """Calculate position accuracy metrics"""
        det_objects = self.detections_to_objects(detections)
        gt_objects = self.detections_to_objects(ground_truth)

        # Calculate position errors
        position_errors = []
        for det in det_objects:
            best_match = None
            best_iou = 0

            # Find best matching ground truth object
            for gt in gt_objects:
                iou = self.calculate_iou(det['bbox'], gt['bbox'])
                if iou > best_iou and iou >= self.iou_threshold:
                    best_iou = iou
                    best_match = gt

            if best_match is not None:
                # Calculate position error
                det_center = np.array([det['bbox']['center_x'], det['bbox']['center_y']])
                gt_center = np.array([best_match['bbox']['center_x'], best_match['bbox']['center_y']])
                error = np.linalg.norm(det_center - gt_center)
                position_errors.append(error)

        # Calculate statistics
        if position_errors:
            mean_error = np.mean(position_errors)
            median_error = np.median(position_errors)
            max_error = np.max(position_errors)
            within_tolerance = sum(1 for e in position_errors if e <= self.position_tolerance)
            accuracy = within_tolerance / len(position_errors)
        else:
            mean_error = 0
            median_error = 0
            max_error = 0
            accuracy = 0

        return {
            'mean_error': mean_error,
            'median_error': median_error,
            'max_error': max_error,
            'accuracy': accuracy,
            'position_errors': position_errors
        }

    def detections_to_objects(self, detection_array):
        """Convert Detection2DArray to list of object dictionaries"""
        objects = []
        for detection in detection_array.detections:
            if detection.results:
                # Get the most confident result
                best_result = max(detection.results, key=lambda x: x.score)

                obj = {
                    'class': best_result.id,
                    'confidence': best_result.score,
                    'bbox': {
                        'center_x': detection.bbox.center.x,
                        'center_y': detection.bbox.center.y,
                        'size_x': detection.bbox.size_x,
                        'size_y': detection.bbox.size_y
                    }
                }
                objects.append(obj)
        return objects

    def calculate_iou(self, bbox1, bbox2):
        """Calculate Intersection over Union between two bounding boxes"""
        # Calculate bounding box coordinates
        x1_min = bbox1['center_x'] - bbox1['size_x'] / 2
        x1_max = bbox1['center_x'] + bbox1['size_x'] / 2
        y1_min = bbox1['center_y'] - bbox1['size_y'] / 2
        y1_max = bbox1['center_y'] + bbox1['size_y'] / 2

        x2_min = bbox2['center_x'] - bbox2['size_x'] / 2
        x2_max = bbox2['center_x'] + bbox2['size_x'] / 2
        y2_min = bbox2['center_y'] - bbox2['size_y'] / 2
        y2_max = bbox2['center_y'] + bbox2['size_y'] / 2

        # Calculate intersection
        inter_x_min = max(x1_min, x2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_min = max(y1_min, y2_min)
        inter_y_max = min(y1_max, y2_max)

        if inter_x_max < inter_x_min or inter_y_max < inter_y_min:
            return 0.0

        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
        bbox1_area = bbox1['size_x'] * bbox1['size_y']
        bbox2_area = bbox2['size_x'] * bbox2['size_y']
        union_area = bbox1_area + bbox2_area - inter_area

        if union_area == 0:
            return 0.0

        return inter_area / union_area

    def generate_accuracy_report(self):
        """Generate and print the accuracy report"""
        self.get_logger().info('=== ACCURACY VERIFICATION REPORT ===')

        # Print overall metrics
        self.get_logger().info(f'Overall Accuracy: {self.results["overall_accuracy"]:.3f}')
        self.get_logger().info(f'Precision: {self.results["precision"]:.3f}')
        self.get_logger().info(f'Recall: {self.results["recall"]:.3f}')
        self.get_logger().info(f'F1-Score: {self.results["f1_score"]:.3f}')
        self.get_logger().info(f'mAP: {self.results["mAP"]:.3f}')

        # Print position accuracy
        if self.results['position_metrics']:
            pos_metrics = self.results['position_metrics'][-1]  # Use last metrics for reporting
            self.get_logger().info(f'Position Accuracy: {pos_metrics["accuracy"]:.3f}')
            self.get_logger().info(f'Mean Position Error: {pos_metrics["mean_error"]:.3f} pixels')
            self.get_logger().info(f'Median Position Error: {pos_metrics["median_error"]:.3f} pixels')
            self.get_logger().info(f'Max Position Error: {pos_metrics["max_error"]:.3f} pixels')

        # Print pass/fail status
        accuracy_pass = self.results['overall_accuracy'] >= 0.7  # 70% threshold
        precision_pass = self.results['precision'] >= 0.7
        recall_pass = self.results['recall'] >= 0.7
        position_pass = self.results['position_metrics'] and self.results['position_metrics'][-1]['accuracy'] >= 0.8

        self.get_logger().info(f'--- PASS/FAIL CRITERIA ---')
        self.get_logger().info(f'  Overall Accuracy (70%): {"✓ PASS" if accuracy_pass else "✗ FAIL"}')
        self.get_logger().info(f'  Precision (70%): {"✓ PASS" if precision_pass else "✗ FAIL"}')
        self.get_logger().info(f'  Recall (70%): {"✓ PASS" if recall_pass else "✗ FAIL"}')
        self.get_logger().info(f'  Position Accuracy (80%): {"✓ PASS" if position_pass else "✗ FAIL"}')

        # Overall assessment
        all_pass = all([accuracy_pass, precision_pass, recall_pass, position_pass])
        self.get_logger().info(f'--- OVERALL ASSESSMENT ---')
        self.get_logger().info(f'  Perception accuracy requirements met: {"✓ PASS" if all_pass else "✗ FAIL"}')

    def save_test_results(self):
        """Save test results to file"""
        results_path = Path(self.results_directory)
        results_path.mkdir(parents=True, exist_ok=True)

        # Prepare results data
        results_data = {
            'timestamp': time.time(),
            'test_duration': self.test_duration,
            'iou_threshold': self.iou_threshold,
            'confidence_threshold': self.confidence_threshold,
            'position_tolerance': self.position_tolerance,
            'metrics': {
                'overall_accuracy': self.results['overall_accuracy'],
                'precision': self.results['precision'],
                'recall': self.results['recall'],
                'f1_score': self.results['f1_score'],
                'mAP': self.results['mAP']
            },
            'detection_metrics': self.results['detection_metrics'],
            'position_metrics': self.results['position_metrics']
        }

        # Save to JSON file
        results_file = results_path / f'accuracy_results_{int(time.time())}.json'
        with open(results_file, 'w') as f:
            json.dump(results_data, f, indent=2)

        self.get_logger().info(f'Test results saved to: {results_file}')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.test_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    verification_node = AccuracyVerificationNode()

    try:
        # Start the test in a separate thread to avoid blocking
        test_thread = threading.Thread(target=verification_node.start_test)
        test_thread.start()

        # Run the node
        rclpy.spin(verification_node)

        # Wait for test to complete
        test_thread.join(timeout=verification_node.test_duration + 10.0)  # Add 10s buffer
    except KeyboardInterrupt:
        verification_node.get_logger().info('Accuracy verification test interrupted by user')
    finally:
        verification_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()