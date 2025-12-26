#!/usr/bin/env python3
"""
Isaac Sim Navigation Bridge Node

This node bridges Isaac Sim with ROS 2 navigation system,
enabling integration between Isaac Sim environment and Nav2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Header
from tf2_msgs.msg import TFMessage, TransformStamped
from builtin_interfaces.msg import Time
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray
from cv_bridge import CvBridge
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R


class NavigationBridgeNode(Node):
    def __init__(self):
        super().__init__('navigation_bridge_node')

        # Declare parameters
        self.declare_parameter('config_file', '/config/navigation_bridge_config.yaml')
        self.declare_parameter('robot_name', 'HumanoidRobot')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_frequency', 60.0)
        self.declare_parameter('use_sim_time', True)

        # Get parameters
        self.config_file = self.get_parameter('config_file').value
        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.use_sim_time = self.get_parameter('use_sim_time').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize robot state variables
        self.current_time = self.get_clock().now()
        self.previous_time = self.get_clock().now()
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.robot_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'torso_joint': 0.0,
            'head_joint': 0.0
        }

        # Navigation state
        self.navigation_goal = None
        self.navigation_active = False
        self.navigation_path = Path()
        self.navigation_status = "IDLE"

        # Data locks
        self.state_lock = threading.Lock()
        self.nav_lock = threading.Lock()

        # Create subscribers
        self.nav_command_sub = self.create_subscription(
            Twist,
            '/isaac_sim/nav_command',
            self.nav_command_callback,
            sensor_qos
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/isaac_sim/goal_pose',
            self.goal_pose_callback,
            sensor_qos
        )

        self.navigate_to_pose_sub = self.create_subscription(
            String,  # Using String as placeholder - in real implementation, use proper action client
            '/navigate_to_pose',
            self.navigate_to_pose_callback,
            sensor_qos
        )

        # Create publishers
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/isaac_sim/joint_states',
            sensor_qos
        )

        self.tf_pub = self.create_publisher(
            TFMessage,
            '/isaac_sim/tf',
            sensor_qos
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            sensor_qos
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/sensors/imu/data',
            sensor_qos
        )

        self.path_pub = self.create_publisher(
            Path,
            '/navigation/path',
            sensor_qos
        )

        self.status_pub = self.create_publisher(
            String,  # Using String as placeholder - in real implementation, use proper status message
            '/navigation/status',
            sensor_qos
        )

        # Create timer for publishing state
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_robot_state)

        self.get_logger().info('Navigation Bridge Node initialized')
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Publish frequency: {self.publish_frequency} Hz')

    def nav_command_callback(self, msg):
        """Handle navigation command from ROS"""
        with self.state_lock:
            # Convert Twist command to robot motion
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            # Update robot velocity based on command
            self.robot_velocity[0] = linear_x
            self.robot_velocity[1] = linear_y
            self.robot_angular_velocity[2] = angular_z

            self.get_logger().debug(f'Received nav command: linear=({linear_x}, {linear_y}), angular=({angular_z})')

    def goal_pose_callback(self, msg):
        """Handle goal pose commands from ROS"""
        with self.nav_lock:
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_z = msg.pose.position.z

            self.navigation_goal = np.array([goal_x, goal_y, goal_z])
            self.navigation_active = True
            self.navigation_status = "ACTIVE"

            self.get_logger().info(f'Received goal pose: ({goal_x}, {goal_y}, {goal_z})')

            # In a real implementation, this would trigger navigation in Isaac Sim

    def navigate_to_pose_callback(self, msg):
        """Handle navigate to pose commands"""
        with self.nav_lock:
            # Parse navigation command
            try:
                nav_data = eval(msg.data) if msg.data else {}

                if 'goal' in nav_data:
                    goal = nav_data['goal']
                    self.navigation_goal = np.array([goal.get('x', 0), goal.get('y', 0), goal.get('z', 0)])
                    self.navigation_active = True
                    self.navigation_status = "ACTIVE"

                    self.get_logger().info(f'Navigation started to goal: {self.navigation_goal}')

                    # Simulate navigation path generation
                    self.generate_navigation_path()

            except Exception as e:
                self.get_logger().error(f'Error parsing navigation command: {e}')

    def generate_navigation_path(self):
        """Generate a simple navigation path for simulation"""
        # This is a simplified path generation for simulation
        # In a real implementation, this would use the Nav2 path planner

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame

        # Generate a simple path from current position to goal
        start_pos = self.robot_position
        goal_pos = self.navigation_goal

        if goal_pos is not None:
            # Create intermediate points along the path
            steps = 10  # Number of intermediate points
            for i in range(steps + 1):
                fraction = i / steps
                x = start_pos[0] + fraction * (goal_pos[0] - start_pos[0])
                y = start_pos[1] + fraction * (goal_pos[1] - start_pos[1])
                z = start_pos[2] + fraction * (goal_pos[2] - start_pos[2])

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = self.map_frame
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z
                pose_stamped.pose.orientation.w = 1.0

                path.poses.append(pose_stamped)

        self.navigation_path = path

    def publish_robot_state(self):
        """Publish robot state to ROS topics"""
        self.current_time = self.get_clock().now()

        # Publish joint states
        self.publish_joint_states()

        # Publish TF transforms
        self.publish_tf()

        # Publish odometry
        self.publish_odometry()

        # Publish IMU data
        self.publish_imu_data()

        # Publish navigation path if available
        if self.navigation_path.poses:
            self.path_pub.publish(self.navigation_path)

        # Publish navigation status
        status_msg = String()
        status_msg.data = self.navigation_status
        self.status_pub.publish(status_msg)

        # Update robot position based on velocity (simple integration)
        dt = (self.current_time.nanoseconds - self.previous_time.nanoseconds) / 1e9
        if dt > 0:
            self.robot_position += self.robot_velocity * dt
            # Update orientation based on angular velocity
            # Simplified: just update yaw for now
            yaw_change = self.robot_angular_velocity[2] * dt
            # Update orientation quaternion based on yaw change
            # This is a simplified approach - in reality, you'd use proper quaternion math
            self.get_logger().debug(f'Robot position: {self.robot_position}')

        self.previous_time = self.current_time

    def publish_joint_states(self):
        """Publish joint state data"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.current_time.to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        # Add joint names and positions
        joint_state_msg.name = list(self.joint_positions.keys())
        joint_state_msg.position = list(self.joint_positions.values())

        # For simplicity, set zero velocity and effort
        joint_state_msg.velocity = [0.0] * len(self.joint_positions)
        joint_state_msg.effort = [0.0] * len(self.joint_positions)

        self.joint_states_pub.publish(joint_state_msg)

    def publish_tf(self):
        """Publish transform data"""
        tf_msg = TFMessage()

        # Create transform from odom to base_link
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = self.current_time.to_msg()
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        odom_to_base.transform.translation.x = float(self.robot_position[0])
        odom_to_base.transform.translation.y = float(self.robot_position[1])
        odom_to_base.transform.translation.z = float(self.robot_position[2])
        odom_to_base.transform.rotation.x = 0.0  # Simplified
        odom_to_base.transform.rotation.y = 0.0
        odom_to_base.transform.rotation.z = 0.0
        odom_to_base.transform.rotation.w = 1.0

        tf_msg.transforms.append(odom_to_base)

        # Add transforms for each joint
        for joint_name in self.joint_positions.keys():
            if 'hip' in joint_name or 'knee' in joint_name or 'ankle' in joint_name:
                link_name = joint_name.replace('_joint', '_link')
                joint_tf = TransformStamped()
                joint_tf.header.stamp = self.current_time.to_msg()
                joint_tf.header.frame_id = 'base_link'
                joint_tf.child_frame_id = link_name
                joint_tf.transform.translation.x = 0.0
                joint_tf.transform.translation.y = 0.0
                joint_tf.transform.translation.z = 0.0
                joint_tf.transform.rotation.x = 0.0
                joint_tf.transform.rotation.y = 0.0
                joint_tf.transform.rotation.z = 0.0
                joint_tf.transform.rotation.w = 1.0
                tf_msg.transforms.append(joint_tf)

        self.tf_pub.publish(tf_msg)

    def publish_odometry(self):
        """Publish odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = float(self.robot_position[0])
        odom_msg.pose.pose.position.y = float(self.robot_position[1])
        odom_msg.pose.pose.position.z = float(self.robot_position[2])

        # Set orientation (simplified)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Set velocity
        odom_msg.twist.twist.linear.x = float(self.robot_velocity[0])
        odom_msg.twist.twist.linear.y = float(self.robot_velocity[1])
        odom_msg.twist.twist.linear.z = float(self.robot_velocity[2])
        odom_msg.twist.twist.angular.x = float(self.robot_angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(self.robot_angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(self.robot_angular_velocity[2])

        self.odom_pub.publish(odom_msg)

    def publish_imu_data(self):
        """Publish IMU sensor data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation (simplified)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        # Set angular velocity
        imu_msg.angular_velocity.x = float(self.robot_angular_velocity[0])
        imu_msg.angular_velocity.y = float(self.robot_angular_velocity[1])
        imu_msg.angular_velocity.z = float(self.robot_angular_velocity[2])

        # Set linear acceleration (simplified - just gravity)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = -9.81

        # Set covariance matrices (set to zero for now)
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    bridge_node = NavigationBridgeNode()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Navigation Bridge Node interrupted by user')
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()