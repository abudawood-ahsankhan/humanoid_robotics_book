#!/usr/bin/env python3

"""
Visual Grounding Node for VLA System

This node links visual information to action parameters.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_msgs.msg import ObjectList, SceneDescription, RecognitionRequest, RecognitionResult
from geometry_msgs.msg import Pose


class VisualGroundingNode(Node):
    def __init__(self):
        super().__init__('visual_grounding_node')

        # Create subscriber for command text
        self.command_sub = self.create_subscription(
            String,
            '/vla/command_text',
            self.command_callback,
            10
        )

        # Create subscriber for detected objects
        self.objects_sub = self.create_subscription(
            ObjectList,
            '/vla/detected_objects',
            self.objects_callback,
            10
        )

        # Create subscriber for scene descriptions
        self.scene_sub = self.create_subscription(
            SceneDescription,
            '/vla/scene_description',
            self.scene_callback,
            10
        )

        # Create subscriber for recognition requests
        self.recognition_request_sub = self.create_subscription(
            RecognitionRequest,
            '/vla/recognition_request',
            self.recognition_request_callback,
            10
        )

        # Create publisher for recognition results
        self.recognition_result_pub = self.create_publisher(RecognitionResult, '/vla/recognition_result', 10)

        # Store scene information
        self.current_objects = []
        self.current_scene = None

        self.get_logger().info('Visual Grounding Node initialized')

    def command_callback(self, msg):
        """Process command text to identify relevant objects"""
        command = msg.data.lower()

        # Extract object information from command
        target_objects = self.extract_target_objects(command)

        # Find matching objects in current scene
        matching_objects = self.find_matching_objects(target_objects)

        # Publish results or trigger actions based on matches
        self.get_logger().info(f'Command "{command}" matched {len(matching_objects)} objects')

    def objects_callback(self, msg):
        """Update current objects"""
        self.current_objects = msg.objects

    def scene_callback(self, msg):
        """Update current scene"""
        self.current_scene = msg

    def recognition_request_callback(self, msg):
        """Handle recognition request"""
        try:
            # Process the recognition request
            result = self.process_recognition_request(msg)

            # Create and publish result
            result_msg = RecognitionResult()
            result_msg.request_id = msg.request_id
            result_msg.recognized_objects = result['objects']
            result_msg.status = result['status']
            result_msg.timestamp = self.get_clock().now().to_msg()
            result_msg.confidence = result['confidence']

            self.recognition_result_pub.publish(result_msg)

            self.get_logger().info(f'Recognition request {msg.request_id} processed')

        except Exception as e:
            self.get_logger().error(f'Error processing recognition request: {e}')

    def extract_target_objects(self, command):
        """Extract target objects from command text"""
        # Simple keyword-based extraction (in real system, would use NLP)
        keywords = ['red', 'blue', 'green', 'cup', 'ball', 'box', 'table', 'chair']
        target_objects = []

        for keyword in keywords:
            if keyword in command:
                target_objects.append(keyword)

        return target_objects

    def find_matching_objects(self, target_objects):
        """Find objects in current scene that match target objects"""
        matches = []

        for obj in self.current_objects:
            for target in target_objects:
                if target in obj.class_name.lower() or target in obj.properties:
                    matches.append(obj)

        return matches

    def process_recognition_request(self, request):
        """Process a recognition request"""
        # Find objects that match the request context
        matching_objects = []

        for obj in self.current_objects:
            if any(obj_type in obj.class_name.lower() for obj_type in request.object_types):
                matching_objects.append(obj)

        return {
            'objects': matching_objects,
            'status': 'success' if matching_objects else 'not_found',
            'confidence': 0.9 if matching_objects else 0.1
        }


def main(args=None):
    rclpy.init(args=args)
    node = VisualGroundingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()