#!/usr/bin/env python3

"""
Scene Analysis Node for VLA System

This node analyzes detected objects for spatial relationships and scene understanding.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import ObjectList, SceneDescription
from geometry_msgs.msg import Pose


class SceneAnalysisNode(Node):
    def __init__(self):
        super().__init__('scene_analysis_node')

        # Create subscriber for detected objects
        self.objects_sub = self.create_subscription(
            ObjectList,
            '/vla/detected_objects',
            self.objects_callback,
            10
        )

        # Create publisher for scene descriptions
        self.scene_pub = self.create_publisher(SceneDescription, '/vla/scene_description', 10)

        # Scene analysis parameters
        self.spatial_relationship_threshold = 0.7
        self.context_awareness_enabled = True

        self.get_logger().info('Scene Analysis Node initialized')

    def objects_callback(self, msg):
        """Process detected objects and analyze scene"""
        try:
            # Analyze the scene based on detected objects
            scene_description = self.analyze_scene(msg.objects)

            # Create and publish scene description
            scene_msg = SceneDescription()
            scene_msg.scene_id = f"scene_{self.get_clock().now().nanoseconds}"
            scene_msg.description = scene_description
            scene_msg.objects = msg.objects
            scene_msg.timestamp = self.get_clock().now().to_msg()
            scene_msg.spatial_relationships = self.compute_spatial_relationships(msg.objects)

            self.scene_pub.publish(scene_msg)

            self.get_logger().info(f'Published scene description with {len(msg.objects)} objects')

        except Exception as e:
            self.get_logger().error(f'Error analyzing scene: {e}')

    def analyze_scene(self, objects):
        """Analyze the scene based on detected objects"""
        if not objects:
            return "Empty scene - no objects detected"

        # Create a description of the scene
        object_types = [obj.class_name for obj in objects]
        unique_types = list(set(object_types))

        description = f"Scene contains {len(objects)} objects of types: {', '.join(unique_types)}. "

        # Add more context based on object positions
        if len(objects) == 1:
            description += f"There is a single {objects[0].class_name} in the scene."
        elif len(objects) > 1:
            description += "Multiple objects are present with various spatial relationships."

        return description

    def compute_spatial_relationships(self, objects):
        """Compute spatial relationships between objects"""
        relationships = []

        if len(objects) < 2:
            return relationships

        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    # Calculate spatial relationship based on positions
                    pos1 = obj1.pose.position
                    pos2 = obj2.pose.position

                    # Calculate distance
                    distance = ((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)**0.5

                    # Determine spatial relationship based on relative positions
                    dx = pos1.x - pos2.x
                    dy = pos1.y - pos2.y
                    dz = pos1.z - pos2.z

                    relationship = f"{obj1.class_name} is "
                    if abs(dx) > abs(dy) and abs(dx) > abs(dz):
                        if dx > 0:
                            relationship += f"to the right of {obj2.class_name}"
                        else:
                            relationship += f"to the left of {obj2.class_name}"
                    elif abs(dy) > abs(dz):
                        if dy > 0:
                            relationship += f"in front of {obj2.class_name}"
                        else:
                            relationship += f"behind {obj2.class_name}"
                    else:
                        if dz > 0:
                            relationship += f"above {obj2.class_name}"
                        else:
                            relationship += f"below {obj2.class_name}"

                    relationship += f" (distance: {distance:.2f})"
                    relationships.append(relationship)

        return relationships


def main(args=None):
    rclpy.init(args=args)
    node = SceneAnalysisNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()