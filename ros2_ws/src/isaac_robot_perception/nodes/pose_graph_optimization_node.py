#!/usr/bin/env python3
"""
Isaac ROS Pose Graph Optimization Node

This node implements pose graph optimization for Visual SLAM applications
to refine robot trajectory and map consistency.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import String, Header
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque, OrderedDict
from scipy.spatial.transform import Rotation as R


class PoseGraphOptimizationNode(Node):
    def __init__(self):
        super().__init__('pose_graph_optimization_node')

        # Declare parameters
        self.declare_parameter('pose_topic', '/vslam/pose')
        self.declare_parameter('loop_closure_topic', '/vslam/loop_closure_matches')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('optimized_pose_topic', '/vslam/pose_optimized')
        self.declare_parameter('optimized_path_topic', '/vslam/path_optimized')
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('convergence_threshold', 1e-6)
        self.declare_parameter('optimization_frequency', 0.1)  # Hz
        self.declare_parameter('robust_kernel', 'cauchy')

        # Get parameters
        self.pose_topic = self.get_parameter('pose_topic').value
        self.loop_closure_topic = self.get_parameter('loop_closure_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.optimized_pose_topic = self.get_parameter('optimized_pose_topic').value
        self.optimized_path_topic = self.get_parameter('optimized_path_topic').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.optimization_frequency = self.get_parameter('optimization_frequency').value
        self.robust_kernel = self.get_parameter('robust_kernel').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Initialize pose graph variables
        self.poses = OrderedDict()  # Store poses with timestamps
        self.constraints = []       # Store pose constraints
        self.optimized_poses = {}   # Store optimized poses
        self.loop_closures = []     # Store loop closure constraints
        self.graph_lock = threading.Lock()

        # Initialize optimization state
        self.needs_optimization = False
        self.last_optimization_time = self.get_clock().now()

        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            sensor_qos
        )

        self.loop_closure_sub = self.create_subscription(
            String,  # Using String for simplicity - in real implementation, use proper message type
            self.loop_closure_topic,
            self.loop_closure_callback,
            sensor_qos
        )

        # Create publishers
        self.optimized_pose_pub = self.create_publisher(
            PoseStamped,
            self.optimized_pose_topic,
            sensor_qos
        )

        self.optimized_path_pub = self.create_publisher(
            Path,
            self.optimized_path_topic,
            sensor_qos
        )

        # Create timer for optimization
        self.optimization_timer = self.create_timer(1.0 / self.optimization_frequency, self.optimize_pose_graph)

        self.get_logger().info('Pose Graph Optimization Node initialized')
        self.get_logger().info(f'Pose topic: {self.pose_topic}')
        self.get_logger().info(f'Optimization frequency: {self.optimization_frequency} Hz')

    def pose_callback(self, msg):
        """Callback for pose messages"""
        with self.graph_lock:
            # Store the new pose
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            pose_id = len(self.poses)

            # Store pose as [x, y, z, qx, qy, qz, qw]
            pose_data = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ])

            self.poses[pose_id] = {
                'timestamp': timestamp,
                'pose': pose_data,
                'msg': msg
            }

            # Add odometry constraint to previous pose (if available)
            if len(self.poses) > 1:
                prev_id = pose_id - 1
                if prev_id in self.poses:
                    # Create relative pose constraint
                    rel_constraint = self.create_relative_constraint(prev_id, pose_id)
                    self.constraints.append(rel_constraint)

            # Mark that optimization is needed
            self.needs_optimization = True

            self.get_logger().debug(f'Added pose {pose_id} to graph')

    def loop_closure_callback(self, msg):
        """Callback for loop closure messages"""
        with self.graph_lock:
            try:
                # In a real implementation, this would be a proper loop closure message
                # For simplicity, we'll parse the string message
                loop_data = eval(msg.data) if msg.data else {}

                if 'current_id' in loop_data and 'candidate_id' in loop_data:
                    loop_closure = {
                        'from_id': loop_data['candidate_id'],
                        'to_id': loop_data['current_id'],
                        'score': loop_data.get('score', 0.0),
                        'timestamp': loop_data.get('timestamp', None)
                    }

                    self.loop_closures.append(loop_closure)
                    self.needs_optimization = True

                    self.get_logger().info(f'Added loop closure: {loop_closure["from_id"]} -> {loop_closure["to_id"]}, score: {loop_closure["score"]:.3f}')

            except Exception as e:
                self.get_logger().error(f'Error processing loop closure: {e}')

    def create_relative_constraint(self, from_id, to_id):
        """Create a relative pose constraint between two poses"""
        from_pose = self.poses[from_id]['pose']
        to_pose = self.poses[to_id]['pose']

        # Calculate relative transformation
        # Convert quaternions to rotation matrices
        from_rot = R.from_quat([from_pose[3], from_pose[4], from_pose[5], from_pose[6]]).as_matrix()
        to_rot = R.from_quat([to_pose[3], to_pose[4], to_pose[5], to_pose[6]]).as_matrix()

        # Calculate relative rotation
        rel_rot = to_rot @ from_rot.T
        rel_quat = R.from_matrix(rel_rot).as_quat()

        # Calculate relative translation
        from_pos = from_pose[:3]
        to_pos = to_pose[:3]
        rel_trans = to_pos - from_pos

        # Create constraint
        constraint = {
            'from_id': from_id,
            'to_id': to_id,
            'relative_pose': np.concatenate([rel_trans, rel_quat]),
            'information_matrix': np.eye(6) * 100  # High confidence for odometry
        }

        return constraint

    def optimize_pose_graph(self):
        """Perform pose graph optimization"""
        if not self.needs_optimization or len(self.poses) < 2:
            return

        with self.graph_lock:
            # Check if enough time has passed since last optimization
            current_time = self.get_clock().now()
            time_since_last = (current_time.nanoseconds - self.last_optimization_time.nanoseconds) / 1e9

            if time_since_last < (1.0 / self.optimization_frequency):
                return

            self.get_logger().info(f'Starting pose graph optimization with {len(self.poses)} poses and {len(self.constraints)} constraints')

            # Perform optimization (simplified implementation)
            # In a real implementation, this would use g2o, Ceres, or GTSAM
            optimized_poses = self.simplified_optimization()

            # Store optimized poses
            self.optimized_poses = optimized_poses
            self.needs_optimization = False
            self.last_optimization_time = current_time

            # Publish optimized results
            self.publish_optimized_results()

            self.get_logger().info(f'Pose graph optimization completed with {len(optimized_poses)} optimized poses')

    def simplified_optimization(self):
        """Simplified optimization implementation"""
        # This is a simplified implementation for demonstration
        # In a real system, this would use proper graph optimization libraries like g2o or Ceres

        optimized_poses = {}

        if not self.poses:
            return optimized_poses

        # Initialize with original poses
        for pose_id, pose_data in self.poses.items():
            optimized_poses[pose_id] = pose_data['pose'].copy()

        # Apply loop closure corrections if available
        for loop_closure in self.loop_closures:
            from_id = loop_closure['from_id']
            to_id = loop_closure['to_id']

            if from_id in optimized_poses and to_id in optimized_poses:
                # Calculate the error between current poses
                from_pose = optimized_poses[from_id]
                to_pose = optimized_poses[to_id]

                # Get the relative transformation that should exist between these poses
                # This is a simplified approach - real optimization would use proper methods
                error_translation = np.array(to_pose[:3]) - np.array(from_pose[:3])
                desired_distance = 0.1  # Example desired distance for loop closure

                # Apply correction (simplified)
                correction = error_translation * 0.1 * loop_closure['score']  # Scale by confidence
                for i in range(from_id, to_id):
                    if i in optimized_poses:
                        optimized_poses[i][:3] -= correction / 2.0

        return optimized_poses

    def publish_optimized_results(self):
        """Publish optimized poses and path"""
        if not self.optimized_poses:
            return

        # Publish the most recent optimized pose
        if self.poses:
            latest_id = max(self.poses.keys())
            if latest_id in self.optimized_poses:
                optimized_pose = self.optimized_poses[latest_id]

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"

                pose_msg.pose.position.x = float(optimized_pose[0])
                pose_msg.pose.position.y = float(optimized_pose[1])
                pose_msg.pose.position.z = float(optimized_pose[2])

                pose_msg.pose.orientation.x = float(optimized_pose[3])
                pose_msg.pose.orientation.y = float(optimized_pose[4])
                pose_msg.pose.orientation.z = float(optimized_pose[5])
                pose_msg.pose.orientation.w = float(optimized_pose[6])

                self.optimized_pose_pub.publish(pose_msg)

        # Publish optimized path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for pose_id in sorted(self.optimized_poses.keys()):
            pose_data = self.optimized_poses[pose_id]

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"

            pose_stamped.pose.position.x = float(pose_data[0])
            pose_stamped.pose.position.y = float(pose_data[1])
            pose_stamped.pose.position.z = float(pose_data[2])

            pose_stamped.pose.orientation.x = float(pose_data[3])
            pose_stamped.pose.orientation.y = float(pose_data[4])
            pose_stamped.pose.orientation.z = float(pose_data[5])
            pose_stamped.pose.orientation.w = float(pose_data[6])

            path_msg.poses.append(pose_stamped)

        self.optimized_path_pub.publish(path_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    optimization_node = PoseGraphOptimizationNode()

    try:
        rclpy.spin(optimization_node)
    except KeyboardInterrupt:
        optimization_node.get_logger().info('Pose Graph Optimization Node interrupted by user')
    finally:
        optimization_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()