#!/usr/bin/env python3
"""
Isaac ROS LiDAR Processing Node

This node processes LiDAR data using Isaac ROS components for
filtering, segmentation, feature extraction, and obstacle detection.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import threading
import time
from collections import deque


class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')

        # Declare parameters
        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('filtered_points_topic', '/perception/lidar/filtered_points')
        self.declare_parameter('clusters_topic', '/perception/lidar/clusters')
        self.declare_parameter('obstacles_topic', '/perception/lidar/obstacles')
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('enable_segmentation', True)
        self.declare_parameter('enable_obstacle_detection', True)
        self.declare_parameter('processing_frequency', 10.0)
        self.declare_parameter('voxel_leaf_size', [0.1, 0.1, 0.1])
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 100)
        self.declare_parameter('max_cluster_size', 25000)

        # Get parameters
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.filtered_points_topic = self.get_parameter('filtered_points_topic').value
        self.clusters_topic = self.get_parameter('clusters_topic').value
        self.obstacles_topic = self.get_parameter('obstacles_topic').value
        self.enable_filtering = self.get_parameter('enable_filtering').value
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.enable_obstacle_detection = self.get_parameter('enable_obstacle_detection').value
        self.processing_frequency = self.get_parameter('processing_frequency').value
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Initialize point cloud processing variables
        self.latest_pointcloud = None
        self.pointcloud_lock = threading.Lock()

        # Create subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos
        )

        # Create publishers
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            self.filtered_points_topic,
            sensor_qos
        )

        self.clusters_pub = self.create_publisher(
            PointCloud2,
            self.clusters_topic,
            sensor_qos
        )

        self.obstacles_pub = self.create_publisher(
            PointCloud2,
            self.obstacles_topic,
            sensor_qos
        )

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.processing_frequency, self.process_pointcloud)

        self.get_logger().info('LiDAR Processing Node initialized')
        self.get_logger().info(f'LiDAR topic: {self.lidar_topic}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')

    def lidar_callback(self, msg):
        """Callback for LiDAR point cloud messages"""
        with self.pointcloud_lock:
            self.latest_pointcloud = msg
            self.get_logger().debug(f'Received point cloud with {msg.width * msg.height} points')

    def process_pointcloud(self):
        """Process the latest point cloud with all enabled processing steps"""
        if self.latest_pointcloud is None:
            return

        with self.pointcloud_lock:
            pointcloud = self.latest_pointcloud

        # Convert ROS PointCloud2 to numpy array
        points = self.pointcloud2_to_array(pointcloud)

        if points is None or len(points) == 0:
            return

        # Step 1: Filtering
        if self.enable_filtering:
            filtered_points = self.filter_pointcloud(points)
        else:
            filtered_points = points

        # Step 2: Ground plane removal
        if self.enable_filtering:
            no_ground_points = self.remove_ground_plane(filtered_points)
        else:
            no_ground_points = filtered_points

        # Step 3: Segmentation
        if self.enable_segmentation:
            clusters = self.segment_pointcloud(no_ground_points)
            # Publish clusters
            if clusters is not None and len(clusters) > 0:
                cluster_msg = self.array_to_pointcloud2(clusters, pointcloud.header)
                self.clusters_pub.publish(cluster_msg)

        # Step 4: Obstacle detection
        if self.enable_obstacle_detection:
            obstacles = self.detect_obstacles(no_ground_points)
            # Publish obstacles
            if obstacles is not None and len(obstacles) > 0:
                obstacle_msg = self.array_to_pointcloud2(obstacles, pointcloud.header)
                self.obstacles_pub.publish(obstacle_msg)

        # Publish filtered point cloud
        if len(filtered_points) > 0:
            filtered_msg = self.array_to_pointcloud2(filtered_points, pointcloud.header)
            self.filtered_pub.publish(filtered_msg)

    def pointcloud2_to_array(self, msg):
        """Convert PointCloud2 message to numpy array"""
        # Get the fields in the point cloud
        field_names = [field.name for field in msg.fields]

        # Determine the size of each point
        point_step = msg.point_step
        width = msg.width
        height = msg.height
        total_points = width * height

        # Parse the binary data
        points = []
        for i in range(0, len(msg.data), point_step):
            point_data = msg.data[i:i+point_step]
            offset = 0

            # Parse each field in the point
            point = {}
            for field in msg.fields:
                if field.datatype == 7:  # FLOAT32
                    value = struct.unpack('<f', point_data[offset:offset+4])[0]
                    offset += 4
                elif field.datatype == 6:  # FLOAT64
                    value = struct.unpack('<d', point_data[offset:offset+8])[0]
                    offset += 8
                else:
                    # For other types, skip
                    continue

                point[field.name] = value

            # Extract x, y, z coordinates
            if 'x' in point and 'y' in point and 'z' in point:
                points.append([point['x'], point['y'], point['z']])

        return np.array(points)

    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2 message"""
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header

        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Set up message parameters
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12  # 3 * 4 bytes (FLOAT32)
        msg.row_step = msg.point_step * len(points)
        msg.height = 1
        msg.width = len(points)

        # Pack the points into binary data
        data = []
        for point in points:
            data.extend([point[0], point[1], point[2]])

        # Convert to bytes
        msg.data = b''.join([struct.pack('<f', val) for val in data])

        return msg

    def filter_pointcloud(self, points):
        """Apply various filters to the point cloud"""
        if len(points) == 0:
            return points

        # Voxel grid filtering (downsampling)
        if self.enable_filtering:
            # Simple voxel grid filtering implementation
            # In a real implementation, this would use Isaac ROS filtering
            filtered_points = self.voxel_grid_filter(points, self.voxel_leaf_size)
        else:
            filtered_points = points

        # Remove points with invalid coordinates (NaN or Inf)
        valid_mask = np.isfinite(points).all(axis=1)
        filtered_points = filtered_points[valid_mask]

        return filtered_points

    def voxel_grid_filter(self, points, leaf_size):
        """Simple voxel grid filtering implementation"""
        # This is a simplified implementation
        # In a real Isaac ROS implementation, this would use optimized CUDA operations
        if len(points) == 0:
            return points

        # Calculate voxel indices
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)

        # Calculate number of voxels in each dimension
        dims = np.ceil((max_vals - min_vals) / leaf_size).astype(int)
        dims = np.maximum(dims, 1)  # Ensure at least 1 voxel in each dimension

        # Create voxel grid
        voxel_grid = {}
        for point in points:
            # Calculate voxel index for this point
            voxel_idx = tuple(((point - min_vals) / leaf_size).astype(int))
            # Ensure voxel index is within bounds
            voxel_idx = tuple(min(i, d-1) for i, d in zip(voxel_idx, dims))

            if voxel_idx not in voxel_grid:
                voxel_grid[voxel_idx] = []
            voxel_grid[voxel_idx].append(point)

        # Take the centroid of each voxel
        filtered_points = []
        for voxel_points in voxel_grid.values():
            if voxel_points:
                centroid = np.mean(voxel_points, axis=0)
                filtered_points.append(centroid)

        return np.array(filtered_points)

    def remove_ground_plane(self, points):
        """Remove ground plane from point cloud using RANSAC-like approach"""
        if len(points) == 0:
            return points

        # Simple ground removal - keep points above z = 0.1 (assuming ground is at z=0)
        # In a real implementation, this would use PCL RANSAC for plane fitting
        ground_threshold = 0.1
        non_ground_mask = points[:, 2] > ground_threshold
        no_ground_points = points[non_ground_mask]

        return no_ground_points

    def segment_pointcloud(self, points):
        """Segment point cloud into clusters"""
        if len(points) == 0:
            return points

        # Simple clustering based on proximity
        # In a real implementation, this would use PCL Euclidean clustering
        if len(points) < self.min_cluster_size:
            return points  # Return as is if too few points

        # For this example, we'll return a subset of points that could represent clusters
        # In real implementation, this would use Isaac ROS segmentation
        if len(points) > self.max_cluster_size:
            # Downsample to max cluster size
            step = len(points) // self.max_cluster_size
            clustered_points = points[::step]
        else:
            clustered_points = points

        return clustered_points

    def detect_obstacles(self, points):
        """Detect obstacles in the point cloud"""
        if len(points) == 0:
            return points

        # Simple obstacle detection - points within a certain height range
        # In a real implementation, this would use Isaac ROS obstacle detection
        min_height = -1.0  # Below ground
        max_height = 1.5   # Up to human height
        height_mask = (points[:, 2] > min_height) & (points[:, 2] < max_height)

        # Also filter by distance from robot (assuming robot is at origin)
        max_distance = 10.0  # 10 meters
        dist_mask = np.sqrt(points[:, 0]**2 + points[:, 1]**2) < max_distance

        obstacle_mask = height_mask & dist_mask
        obstacles = points[obstacle_mask]

        return obstacles

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarProcessingNode()

    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        lidar_node.get_logger().info('LiDAR Processing Node interrupted by user')
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()