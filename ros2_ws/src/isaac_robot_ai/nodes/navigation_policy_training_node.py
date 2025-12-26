#!/usr/bin/env python3
"""
Isaac Sim Navigation Policy Training Node

This node implements navigation policy training using Isaac Sim
for reinforcement learning-based navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
import numpy as np
import threading
import time
from collections import deque
import random
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter


class NavigationPolicyNetwork(nn.Module):
    """Simple neural network for navigation policy"""
    def __init__(self, input_dim, output_dim, hidden_dims=[256, 256, 128]):
        super(NavigationPolicyNetwork, self).__init__()

        layers = []
        prev_dim = input_dim

        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, hidden_dim))
            layers.append(nn.ReLU())
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, output_dim))

        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)


class NavigationPolicyTrainingNode(Node):
    def __init__(self):
        super().__init__('navigation_policy_training_node')

        # Declare parameters
        self.declare_parameter('training_config_file', '/config/navigation_policy_training_config.yaml')
        self.declare_parameter('robot_name', 'HumanoidRobot')
        self.declare_parameter('episode_max_steps', 1000)
        self.declare_parameter('success_reward', 100.0)
        self.declare_parameter('collision_penalty', -50.0)
        self.declare_parameter('time_penalty', -0.1)
        self.declare_parameter('distance_reward', 0.1)
        self.declare_parameter('learning_rate', 3e-4)
        self.declare_parameter('gamma', 0.99)
        self.declare_parameter('training_enabled', True)
        self.declare_parameter('model_save_path', '/tmp/navigation_policy.pth')

        # Get parameters
        self.training_config_file = self.get_parameter('training_config_file').value
        self.robot_name = self.get_parameter('robot_name').value
        self.episode_max_steps = self.get_parameter('episode_max_steps').value
        self.success_reward = self.get_parameter('success_reward').value
        self.collision_penalty = self.get_parameter('collision_penalty').value
        self.time_penalty = self.get_parameter('time_penalty').value
        self.distance_reward = self.get_parameter('distance_reward').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.gamma = self.get_parameter('gamma').value
        self.training_enabled = self.get_parameter('training_enabled').value
        self.model_save_path = self.get_parameter('model_save_path').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize training variables
        self.current_episode = 0
        self.current_step = 0
        self.episode_reward = 0.0
        self.total_reward = 0.0
        self.episode_losses = deque(maxlen=100)
        self.episode_rewards = deque(maxlen=100)
        self.training_active = False
        self.episode_active = False

        # Robot state variables
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self.robot_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {}
        self.goal_position = np.array([5.0, 5.0, 0.0])  # Default goal

        # Training state
        self.current_observation = None
        self.previous_observation = None
        self.current_action = None
        self.previous_action = None
        self.reward = 0.0
        self.done = False
        self.collision_detected = False

        # Neural network components
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.observation_dim = 10  # Simplified observation space
        self.action_dim = 2  # linear velocity, angular velocity
        self.policy_network = NavigationPolicyNetwork(self.observation_dim, self.action_dim)
        self.optimizer = optim.Adam(self.policy_network.parameters(), lr=self.learning_rate)
        self.loss_fn = nn.MSELoss()

        # Data collection
        self.observations = deque(maxlen=10000)
        self.actions = deque(maxlen=10000)
        self.rewards = deque(maxlen=10000)
        self.next_observations = deque(maxlen=10000)
        self.terminals = deque(maxlen=10000)

        # Locks
        self.state_lock = threading.Lock()
        self.training_lock = threading.Lock()

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/isaac_sim/joint_states',
            self.joint_state_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/sensors/camera/image_raw',
            self.camera_callback,
            sensor_qos
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/sensors/lidar/points',
            self.lidar_callback,
            sensor_qos
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav',
            sensor_qos
        )

        self.training_status_pub = self.create_publisher(
            String,
            '/training/status',
            sensor_qos
        )

        self.episode_info_pub = self.create_publisher(
            String,
            '/training/episode_info',
            sensor_qos
        )

        # Create timer for training loop
        self.training_timer = self.create_timer(0.1, self.training_step)  # 10 Hz training updates

        # Initialize TensorBoard writer
        self.writer = SummaryWriter(log_dir='/tmp/navigation_training_logs')

        self.get_logger().info('Navigation Policy Training Node initialized')
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Training enabled: {self.training_enabled}')
        self.get_logger().info(f'Device: {self.device}')

    def odom_callback(self, msg):
        """Callback for odometry messages"""
        with self.state_lock:
            # Extract position
            self.robot_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])

            # Extract orientation
            self.robot_orientation = np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])

            # Extract velocity
            self.robot_velocity = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])

            self.robot_angular_velocity = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        with self.state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        with self.state_lock:
            # Could be used for balance feedback in real implementation
            pass

    def camera_callback(self, msg):
        """Callback for camera messages"""
        with self.state_lock:
            # Process camera data for observations
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                # In a real implementation, this would extract visual features
            except Exception as e:
                self.get_logger().error(f'Error processing camera image: {e}')

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        with self.state_lock:
            # Process LiDAR data for collision detection
            # This is a simplified approach - in reality, you'd parse the PointCloud2 message
            self.check_collision_from_lidar()

    def check_collision_from_lidar(self):
        """Check for collisions using LiDAR data"""
        # Simplified collision detection
        # In a real implementation, this would parse the PointCloud2 message
        # and check for points within a collision threshold
        self.collision_detected = False

    def get_observation(self):
        """Get current observation from robot state"""
        with self.state_lock:
            # Create a simplified observation vector
            # In a real implementation, this would include sensor data
            obs = np.zeros(self.observation_dim)

            # Robot position relative to goal
            dist_to_goal = np.linalg.norm(self.goal_position[:2] - self.robot_position[:2])
            obs[0] = dist_to_goal
            obs[1] = self.goal_position[0] - self.robot_position[0]  # x diff
            obs[2] = self.goal_position[1] - self.robot_position[1]  # y diff

            # Robot velocity
            obs[3] = self.robot_velocity[0]  # linear x
            obs[4] = self.robot_velocity[1]  # linear y
            obs[5] = self.robot_angular_velocity[2]  # angular z

            # Simplified LiDAR readings (front, left, right)
            obs[6] = 2.0  # front distance (simplified)
            obs[7] = 2.0  # left distance
            obs[8] = 2.0  # right distance

            # Previous action (if available)
            if self.previous_action is not None:
                obs[9] = self.previous_action[1]  # previous angular velocity

            return obs

    def calculate_reward(self):
        """Calculate reward based on current state"""
        with self.state_lock:
            reward = 0.0

            # Distance to goal reward
            dist_to_goal = np.linalg.norm(self.goal_position[:2] - self.robot_position[:2])
            if dist_to_goal < 0.5:  # Reached goal
                reward += self.success_reward
                self.done = True
            else:
                # Negative distance reward (encourage getting closer)
                reward += self.distance_reward * (10.0 - min(dist_to_goal, 10.0))

            # Time penalty
            reward += self.time_penalty

            # Collision penalty
            if self.collision_detected:
                reward += self.collision_penalty
                self.done = True
                self.collision_detected = False

            return reward

    def select_action(self, observation):
        """Select action using the policy network"""
        obs_tensor = torch.FloatTensor(observation).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.policy_network(obs_tensor)

        # Convert to numpy and add some exploration noise
        action_np = action.cpu().numpy()[0]
        action_np += np.random.normal(0, 0.1, size=action_np.shape)  # Exploration noise

        # Clamp actions to valid ranges
        action_np[0] = np.clip(action_np[0], -0.5, 0.5)  # linear velocity
        action_np[1] = np.clip(action_np[1], -0.8, 0.8)  # angular velocity

        return action_np

    def publish_action(self, action):
        """Publish action to robot"""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(action[0])
        cmd_vel_msg.angular.z = float(action[1])

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def training_step(self):
        """Perform one step of training"""
        if not self.training_enabled:
            return

        with self.training_lock:
            # Get current observation
            current_obs = self.get_observation()
            self.current_observation = current_obs

            # Calculate reward for previous action
            if self.previous_observation is not None and self.previous_action is not None:
                self.reward = self.calculate_reward()
                self.episode_reward += self.reward

                # Store transition for training
                self.observations.append(self.previous_observation)
                self.actions.append(self.previous_action)
                self.rewards.append(self.reward)
                self.next_observations.append(current_obs)
                self.terminals.append(self.done)

                # Perform training update if we have enough data
                if len(self.observations) >= 64:  # Batch size
                    self.update_policy()

            # Select new action
            action = self.select_action(current_obs)
            self.current_action = action

            # Publish action
            self.publish_action(action)

            # Update state
            self.previous_observation = current_obs.copy()
            self.previous_action = action.copy()

            # Check if episode should end
            if self.done or self.current_step >= self.episode_max_steps:
                self.end_episode()

            self.current_step += 1

    def update_policy(self):
        """Update the policy network using collected experiences"""
        if len(self.observations) < 64:
            return

        # Sample a batch of experiences
        batch_size = 64
        indices = np.random.choice(len(self.observations), batch_size, replace=False)

        obs_batch = torch.FloatTensor([self.observations[i] for i in indices]).to(self.device)
        action_batch = torch.FloatTensor([self.actions[i] for i in indices]).to(self.device)
        reward_batch = torch.FloatTensor([self.rewards[i] for i in indices]).to(self.device).unsqueeze(1)
        next_obs_batch = torch.FloatTensor([self.next_observations[i] for i in indices]).to(self.device)
        terminal_batch = torch.BoolTensor([self.terminals[i] for i in indices]).to(self.device).unsqueeze(1)

        # Compute target values
        with torch.no_grad():
            next_actions = self.policy_network(next_obs_batch)
            target = reward_batch + self.gamma * next_actions * (~terminal_batch)

        # Compute current predictions
        current_predictions = self.policy_network(obs_batch)

        # Compute loss
        loss = self.loss_fn(current_predictions, target.detach())

        # Backpropagate
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Store loss for monitoring
        self.episode_losses.append(loss.item())

    def start_episode(self):
        """Start a new training episode"""
        self.current_step = 0
        self.episode_reward = 0.0
        self.done = False
        self.episode_active = True
        self.current_episode += 1

        self.get_logger().info(f'Starting episode {self.current_episode}')

    def end_episode(self):
        """End the current episode and reset"""
        self.episode_active = False
        self.episode_rewards.append(self.episode_reward)

        self.get_logger().info(f'Episode {self.current_episode} ended - Reward: {self.episode_reward:.2f}')

        # Publish episode info
        episode_info = String()
        episode_info.data = f"Episode: {self.current_episode}, Reward: {self.episode_reward:.2f}, Steps: {self.current_step}"
        self.episode_info_pub.publish(episode_info)

        # Log to TensorBoard
        if self.writer:
            self.writer.add_scalar('Episode/Reward', self.episode_reward, self.current_episode)
            if self.episode_losses:
                self.writer.add_scalar('Episode/AvgLoss', np.mean(self.episode_losses), self.current_episode)

        # Reset for next episode
        self.current_step = 0
        self.done = False
        self.previous_observation = None
        self.previous_action = None
        self.start_episode()

    def save_model(self):
        """Save the trained model"""
        torch.save(self.policy_network.state_dict(), self.model_save_path)
        self.get_logger().info(f'Model saved to {self.model_save_path}')

    def load_model(self):
        """Load a pre-trained model"""
        try:
            self.policy_network.load_state_dict(torch.load(self.model_save_path, map_location=self.device))
            self.get_logger().info(f'Model loaded from {self.model_save_path}')
        except Exception as e:
            self.get_logger().warn(f'Could not load model from {self.model_save_path}: {e}')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        # Save model before shutting down
        if self.training_enabled:
            self.save_model()

        # Close TensorBoard writer
        if self.writer:
            self.writer.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    training_node = NavigationPolicyTrainingNode()

    try:
        # Load pre-trained model if available
        training_node.load_model()

        # Start first episode
        training_node.start_episode()

        rclpy.spin(training_node)
    except KeyboardInterrupt:
        training_node.get_logger().info('Navigation Policy Training Node interrupted by user')
    finally:
        training_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()