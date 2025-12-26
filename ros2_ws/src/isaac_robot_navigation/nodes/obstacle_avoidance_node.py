#!/usr/bin/env python3
"""
Isaac ROS Obstacle Avoidance Node

This node implements obstacle avoidance using Isaac ROS sensor data
for humanoid robot navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu, LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist
import struct


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/sensors/camera/image_raw')
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('modified_cmd_vel_topic', '/cmd_vel_avoided')
        self.declare_parameter('obstacle_markers_topic', '/navigation/obstacle_markers')
        self.declare_parameter('collision_warning_topic', '/navigation/collision_warning')
        self.declare_parameter('obstacle_map_topic', '/navigation/obstacle_map')
        self.declare_parameter('enable_vision_based', True)
        self.declare_parameter('enable_lidar_based', True)
        self.declare_parameter('obstacle_distance_threshold', 2.0)
        self.declare_parameter('min_obstacle_size', 100)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('processing_frequency', 10.0)

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel').value
        self.modified_cmd_vel_topic = self.get_parameter('modified_cmd_vel_topic').value
        self.obstacle_markers_topic = self.get_parameter('obstacle_markers_topic').value
        self.collision_warning_topic = self.get_parameter('collision_warning_topic').value
        self.obstacle_map_topic = self.get_parameter('obstacle_map_topic').value
        self.enable_vision_based = self.get_parameter('enable_vision_based').value
        self.enable_lidar_based = self.get_parameter('enable_lidar_based').value
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.min_obstacle_size = self.get_parameter('min_obstacle_size').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.processing_frequency = self.get_parameter('processing_frequency').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize sensor data
        self.latest_image = None
        self.latest_pointcloud = None
        self.latest_imu = None
        self.current_cmd_vel = Twist()
        self.obstacles = []
        self.obstacle_markers = MarkerArray()
        self.data_lock = threading.Lock()

        # Initialize avoidance state
        self.avoidance_active = False
        self.avoidance_direction = 0.0  # -1.0 for left, 1.0 for right

        # Create subscribers
        if self.enable_vision_based:
            self.camera_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                sensor_qos
            )

        if self.enable_lidar_based:
            self.lidar_sub = self.create_subscription(
                PointCloud2,
                self.lidar_topic,
                self.lidar_callback,
                sensor_qos
            )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            sensor_qos
        )

        # Create publishers
        self.modified_cmd_vel_pub = self.create_publisher(
            Twist,
            self.modified_cmd_vel_topic,
            sensor_qos
        )

        self.obstacle_markers_pub = self.create_publisher(
            MarkerArray,
            self.obstacle_markers_topic,
            sensor_qos
        )

        self.collision_warning_pub = self.create_publisher(
            Marker,
            self.collision_warning_topic,
            sensor_qos
        )

        self.obstacle_map_pub = self.create_publisher(
            OccupancyGrid,
            self.obstacle_map_topic,
            sensor_qos
        )

        # Create timer for obstacle avoidance processing
        self.avoidance_timer = self.create_timer(1.0 / self.processing_frequency, self.process_obstacle_avoidance)

        self.get_logger().info('Obstacle Avoidance Node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'LiDAR topic: {self.lidar_topic}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')

    def camera_callback(self, msg):
        """Callback for camera messages"""
        if not self.enable_vision_based:
            return

        with self.data_lock:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_image = cv_image.copy()
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        if not self.enable_lidar_based:
            return

        with self.data_lock:
            self.latest_pointcloud = msg

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        with self.data_lock:
            self.latest_imu = msg

    def cmd_vel_callback(self, msg):
        """Callback for velocity command messages"""
        with self.data_lock:
            self.current_cmd_vel = msg

    def process_obstacle_avoidance(self):
        """Process sensor data and generate avoidance commands"""
        with self.data_lock:
            # Process LiDAR data for obstacle detection
            if self.latest_pointcloud is not None:
                obstacles = self.process_pointcloud(self.latest_pointcloud)
                self.obstacles = obstacles

            # Process camera data for obstacle detection
            if self.latest_image is not None and self.enable_vision_based:
                vision_obstacles = self.process_vision(self.latest_image)
                # Combine with LiDAR obstacles
                self.obstacles.extend(vision_obstacles)

        # Generate avoidance commands based on detected obstacles
        modified_cmd_vel = self.generate_avoidance_commands()

        # Publish modified velocity commands
        self.modified_cmd_vel_pub.publish(modified_cmd_vel)

        # Publish obstacle markers for visualization
        self.publish_obstacle_markers()

        # Publish obstacle map
        self.publish_obstacle_map()

    def process_pointcloud(self, pointcloud_msg):
        """Process point cloud data to detect obstacles"""
        # Convert PointCloud2 to numpy array
        obstacles = []

        # Parse the binary data
        points = []
        point_step = pointcloud_msg.point_step
        width = pointcloud_msg.width
        height = pointcloud_msg.height

        for i in range(0, len(pointcloud_msg.data), point_step):
            point_data = pointcloud_msg.data[i:i+point_step]
            offset = 0

            # Parse x, y, z coordinates
            x = struct.unpack('<f', point_data[offset:offset+4])[0]
            offset += 4
            y = struct.unpack('<f', point_data[offset:offset+4])[0]
            offset += 4
            z = struct.unpack('<f', point_data[offset:offset+4])[0]

            # Only consider points in front of the robot
            if 0.1 < x < self.obstacle_distance_threshold and abs(y) < 1.0:
                points.append([x, y, z])

        # Convert to numpy array
        points = np.array(points)

        if len(points) > 0:
            # Cluster points to identify individual obstacles
            obstacles = self.cluster_points(points)

        return obstacles

    def process_vision(self, image):
        """Process vision data to detect obstacles"""
        # This is a simplified vision processing
        # In a real implementation, this would use Isaac ROS segmentation
        obstacles = []

        # For simulation, we'll create some obstacles based on image features
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect edges
        edges = cv2.Canny(gray, 50, 150)

        # Find contours (potential obstacles)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_obstacle_size:
                # Calculate center of contour
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])

                    # Convert to 3D position (simplified)
                    # In a real implementation, this would use depth information
                    distance = 1.0  # Placeholder distance
                    angle = (cx - image.shape[1]/2) * 0.01  # Simplified angle calculation
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)

                    obstacles.append({'position': [x, y, 0.0], 'size': area})

        return obstacles

    def cluster_points(self, points):
        """Cluster points to identify individual obstacles"""
        if len(points) == 0:
            return []

        # Simple clustering based on proximity
        obstacles = []
        visited = set()

        for i, point in enumerate(points):
            if i in visited:
                continue

            # Find nearby points
            nearby_indices = []
            for j, other_point in enumerate(points):
                if j in visited:
                    continue

                distance = np.linalg.norm(point - other_point)
                if distance < 0.5:  # Cluster distance threshold
                    nearby_indices.append(j)

            # Mark these points as visited
            visited.update(nearby_indices)

            # Calculate cluster center
            cluster_points = points[nearby_indices]
            if len(cluster_points) > 0:
                center = np.mean(cluster_points, axis=0)

                # Calculate cluster size
                size = len(cluster_points)

                obstacles.append({
                    'position': center,
                    'size': size,
                    'radius': np.std(cluster_points, axis=0)[:2].mean()  # Average std as radius
                })

        return obstacles

    def generate_avoidance_commands(self):
        """Generate avoidance commands based on detected obstacles"""
        cmd_vel = Twist()

        with self.data_lock:
            # Copy current command
            cmd_vel.linear.x = self.current_cmd_vel.linear.x
            cmd_vel.linear.y = self.current_cmd_vel.linear.y
            cmd_vel.linear.z = self.current_cmd_vel.linear.z
            cmd_vel.angular.x = self.current_cmd_vel.angular.x
            cmd_vel.angular.y = self.current_cmd_vel.angular.y
            cmd_vel.angular.z = self.current_cmd_vel.angular.z

        # Check for obstacles in the path
        obstacles_in_path = []
        for obstacle in self.obstacles:
            pos = obstacle['position']
            # Check if obstacle is in front of the robot
            if 0.1 < pos[0] < 1.0 and abs(pos[1]) < 0.8:  # In front, within robot width
                obstacles_in_path.append(obstacle)

        if obstacles_in_path:
            # Activate avoidance behavior
            self.avoidance_active = True

            # Determine avoidance direction based on obstacle positions
            left_obstacles = [obs for obs in obstacles_in_path if obs['position'][1] < 0]
            right_obstacles = [obs for obs in obstacles_in_path if obs['position'][1] >= 0]

            # Prefer the side with fewer obstacles
            if len(left_obstacles) < len(right_obstacles):
                self.avoidance_direction = -1.0  # Turn left
            else:
                self.avoidance_direction = 1.0   # Turn right

            # Adjust velocity based on obstacle proximity
            closest_obstacle = min(obstacles_in_path, key=lambda x: x['position'][0])
            distance_to_obstacle = closest_obstacle['position'][0]

            # Slow down as we get closer to obstacles
            slowdown_factor = max(0.1, distance_to_obstacle / 1.0)

            # Modify linear velocity
            cmd_vel.linear.x *= slowdown_factor
            cmd_vel.linear.x = min(cmd_vel.linear.x, self.max_linear_speed * slowdown_factor)

            # Add angular velocity for avoidance
            cmd_vel.angular.z = self.avoidance_direction * self.max_angular_speed * (1.0 - slowdown_factor)
        else:
            # No obstacles, use original command (within limits)
            self.avoidance_active = False
            cmd_vel.linear.x = min(abs(cmd_vel.linear.x), self.max_linear_speed) * np.sign(cmd_vel.linear.x)
            cmd_vel.angular.z = min(abs(cmd_vel.angular.z), self.max_angular_speed) * np.sign(cmd_vel.angular.z)

        return cmd_vel

    def publish_obstacle_markers(self):
        """Publish obstacle markers for visualization"""
        marker_array = MarkerArray()

        with self.data_lock:
            for i, obstacle in enumerate(self.obstacles):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "obstacles"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Position
                marker.pose.position.x = obstacle['position'][0]
                marker.pose.position.y = obstacle['position'][1]
                marker.pose.position.z = obstacle['position'][2]
                marker.pose.orientation.w = 1.0

                # Scale
                radius = max(0.1, obstacle.get('radius', 0.2))
                marker.scale.x = radius * 2
                marker.scale.y = radius * 2
                marker.scale.z = 0.2  # Flat marker

                # Color
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.obstacle_markers_pub.publish(marker_array)

    def publish_obstacle_map(self):
        """Publish obstacle map"""
        # Create a simple occupancy grid based on detected obstacles
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        # Map parameters
        map_msg.info.resolution = 0.1  # 10cm resolution
        map_msg.info.width = 200       # 20m x 20m map
        map_msg.info.height = 200
        map_msg.info.origin.position.x = -10.0  # Map centered on robot
        map_msg.info.origin.position.y = -10.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Initialize map with unknown values (-1)
        map_data = [-1] * (map_msg.info.width * map_msg.info.height)

        # Mark detected obstacles
        with self.data_lock:
            for obstacle in self.obstacles:
                x, y, _ = obstacle['position']

                # Convert world coordinates to map indices
                map_x = int((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
                map_y = int((y - map_msg.info.origin.position.y) / map_msg.info.resolution)

                # Check bounds
                if 0 <= map_x < map_msg.info.width and 0 <= map_y < map_msg.info.height:
                    idx = map_y * map_msg.info.width + map_x
                    map_data[idx] = 100  # Mark as occupied

        map_msg.data = map_data
        self.obstacle_map_pub.publish(map_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    avoidance_node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(avoidance_node)
    except KeyboardInterrupt:
        avoidance_node.get_logger().info('Obstacle Avoidance Node interrupted by user')
    finally:
        avoidance_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()