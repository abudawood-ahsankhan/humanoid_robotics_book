#!/usr/bin/env python3
"""
Isaac Sim Domain Randomization Node

This node implements domain randomization for sim-to-real transfer
using Isaac Sim environment randomization techniques.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque
import random
import json
from scipy.spatial.transform import Rotation as R


class DomainRandomizationNode(Node):
    def __init__(self):
        super().__init__('domain_randomization_node')

        # Declare parameters
        self.declare_parameter('config_file', '/config/domain_randomization_config.yaml')
        self.declare_parameter('randomization_enabled', True)
        self.declare_parameter('randomization_frequency', 1.0)  # Hz
        self.declare_parameter('lighting_enabled', True)
        self.declare_parameter('texture_enabled', True)
        self.declare_parameter('physics_enabled', True)
        self.declare_parameter('sensor_enabled', True)
        self.declare_parameter('severity_initial', 0.1)
        self.declare_parameter('severity_max', 0.8)
        self.declare_parameter('severity_growth_rate', 0.0001)

        # Get parameters
        self.config_file = self.get_parameter('config_file').value
        self.randomization_enabled = self.get_parameter('randomization_enabled').value
        self.randomization_frequency = self.get_parameter('randomization_frequency').value
        self.lighting_enabled = self.get_parameter('lighting_enabled').value
        self.texture_enabled = self.get_parameter('texture_enabled').value
        self.physics_enabled = self.get_parameter('physics_enabled').value
        self.sensor_enabled = self.get_parameter('sensor_enabled').value
        self.severity_initial = self.get_parameter('severity_initial').value
        self.severity_max = self.get_parameter('severity_max').value
        self.severity_growth_rate = self.get_parameter('severity_growth_rate').value

        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize randomization variables
        self.current_severity = self.severity_initial
        self.step_count = 0
        self.randomization_lock = threading.Lock()

        # Randomization state
        self.lighting_params = {}
        self.texture_params = {}
        self.physics_params = {}
        self.sensor_params = {}
        self.robot_params = {}

        # Curriculum learning state
        self.curriculum_stage = 0
        self.success_rate_history = deque(maxlen=100)

        # Create publishers
        self.randomization_status_pub = self.create_publisher(
            String,
            '/domain_randomization/status',
            sensor_qos
        )

        self.randomization_params_pub = self.create_publisher(
            String,
            '/domain_randomization/parameters',
            sensor_qos
        )

        self.lighting_params_pub = self.create_publisher(
            String,
            '/domain_randomization/lighting',
            sensor_qos
        )

        self.physics_params_pub = self.create_publisher(
            String,
            '/domain_randomization/physics',
            sensor_qos
        )

        self.sensor_params_pub = self.create_publisher(
            String,
            '/domain_randomization/sensor',
            sensor_qos
        )

        # Create timer for randomization updates
        self.randomization_timer = self.create_timer(1.0 / self.randomization_frequency, self.apply_randomization)

        # Initialize randomization parameters
        self.initialize_randomization_parameters()

        self.get_logger().info('Domain Randomization Node initialized')
        self.get_logger().info(f'Randomization enabled: {self.randomization_enabled}')
        self.get_logger().info(f'Frequency: {self.randomization_frequency} Hz')
        self.get_logger().info(f'Initial severity: {self.severity_initial}')

    def initialize_randomization_parameters(self):
        """Initialize domain randomization parameters"""
        with self.randomization_lock:
            # Initialize lighting parameters
            self.lighting_params = {
                'ambient_intensity': self.severity_initial * 0.8 + 0.2,  # Range [0.2, 1.0]
                'directional_intensity': self.severity_initial * 1000 + 500,  # Range [500, 1500]
                'light_color_temp': self.severity_initial * 3000 + 5000,  # Range [5000, 8000] K
                'shadow_softness': self.severity_initial * 1.5 + 0.5  # Range [0.5, 2.0]
            }

            # Initialize texture parameters
            self.texture_params = {
                'floor_albedo_jitter': [self.severity_initial * 0.1] * 3 + [0.0],
                'floor_roughness': self.severity_initial * 0.8 + 0.1,  # Range [0.1, 0.9]
                'floor_metallic': self.severity_initial * 0.2,  # Range [0.0, 0.2]
                'object_albedo_jitter': [self.severity_initial * 0.15] * 3 + [0.0],
                'object_roughness': self.severity_initial * 0.6 + 0.2  # Range [0.2, 0.8]
            }

            # Initialize physics parameters
            self.physics_params = {
                'friction_static': self.severity_initial * 0.6 + 0.3,  # Range [0.3, 0.9]
                'friction_dynamic': self.severity_initial * 0.6 + 0.2,  # Range [0.2, 0.8]
                'mass_multiplier': self.severity_initial * 0.4 + 0.8,  # Range [0.8, 1.2]
                'joint_damping_multiplier': self.severity_initial * 1.5 + 0.5  # Range [0.5, 2.0]
            }

            # Initialize sensor parameters
            self.sensor_params = {
                'camera_noise_stddev': self.severity_initial * 0.009 + 0.001,  # Range [0.001, 0.01]
                'lidar_noise_stddev': self.severity_initial * 0.019 + 0.001,  # Range [0.001, 0.02]
                'imu_acc_noise_density': self.severity_initial * 0.019 + 0.001,  # Range [0.001, 0.02]
                'camera_distortion_k1': self.severity_initial * 0.2 - 0.1,  # Range [-0.1, 0.1]
                'camera_distortion_k2': self.severity_initial * 0.2 - 0.1   # Range [-0.1, 0.1]
            }

            # Initialize robot parameters
            self.robot_params = {
                'joint_stiffness_multiplier': self.severity_initial * 0.4 + 0.8,  # Range [0.8, 1.2]
                'joint_damping_multiplier': self.severity_initial * 1.0 + 0.5,   # Range [0.5, 1.5]
                'balance_kp_multiplier': self.severity_initial * 0.4 + 0.8,     # Range [0.8, 1.2]
                'balance_kd_multiplier': self.severity_initial * 0.4 + 0.8      # Range [0.8, 1.2]
            }

    def apply_randomization(self):
        """Apply domain randomization to Isaac Sim environment"""
        if not self.randomization_enabled:
            return

        with self.randomization_lock:
            # Update severity based on step count and growth rate
            self.current_severity = min(self.severity_max,
                                      self.severity_initial + self.step_count * self.severity_growth_rate)

            # Apply randomization based on current severity
            self.apply_lighting_randomization()
            self.apply_texture_randomization()
            self.apply_physics_randomization()
            self.apply_sensor_randomization()
            self.apply_robot_randomization()

            # Publish randomization status
            status_msg = String()
            status_msg.data = f"Severity: {self.current_severity:.3f}, Stage: {self.curriculum_stage}"
            self.randomization_status_pub.publish(status_msg)

            # Publish current parameters
            params_msg = String()
            params_msg.data = json.dumps({
                'step': self.step_count,
                'severity': self.current_severity,
                'lighting': self.lighting_params,
                'physics': self.physics_params,
                'sensor': self.sensor_params,
                'robot': self.robot_params
            })
            self.randomization_params_pub.publish(params_msg)

            self.step_count += 1

    def apply_lighting_randomization(self):
        """Apply lighting randomization"""
        if not self.lighting_enabled:
            return

        # Add random variations to lighting parameters based on current severity
        base_ambient = 0.2 + 0.8 * self.severity_initial
        variation = 0.1 * self.current_severity * random.uniform(-1, 1)
        self.lighting_params['ambient_intensity'] = max(0.2, min(1.0, base_ambient + variation))

        base_directional = 500 + 1000 * self.severity_initial
        variation = 200 * self.current_severity * random.uniform(-1, 1)
        self.lighting_params['directional_intensity'] = max(500, min(1500, base_directional + variation))

        base_color_temp = 5000 + 3000 * self.severity_initial
        variation = 500 * self.current_severity * random.uniform(-1, 1)
        self.lighting_params['light_color_temp'] = max(5000, min(8000, base_color_temp + variation))

        base_shadow = 0.5 + 1.5 * self.severity_initial
        variation = 0.3 * self.current_severity * random.uniform(-1, 1)
        self.lighting_params['shadow_softness'] = max(0.5, min(2.0, base_shadow + variation))

        # Publish lighting parameters
        lighting_msg = String()
        lighting_msg.data = json.dumps(self.lighting_params)
        self.lighting_params_pub.publish(lighting_msg)

    def apply_texture_randomization(self):
        """Apply texture randomization"""
        if not self.texture_enabled:
            return

        # Add random variations to texture parameters
        base_floor_roughness = 0.1 + 0.8 * self.severity_initial
        variation = 0.1 * self.current_severity * random.uniform(-1, 1)
        self.texture_params['floor_roughness'] = max(0.1, min(0.9, base_floor_roughness + variation))

        base_floor_metallic = 0.0 + 0.2 * self.severity_initial
        variation = 0.05 * self.current_severity * random.uniform(-1, 1)
        self.texture_params['floor_metallic'] = max(0.0, min(0.2, base_floor_metallic + variation))

        base_obj_roughness = 0.2 + 0.6 * self.severity_initial
        variation = 0.1 * self.current_severity * random.uniform(-1, 1)
        self.texture_params['object_roughness'] = max(0.2, min(0.8, base_obj_roughness + variation))

        # Update albedo jitters based on severity
        jitter_factor = self.current_severity
        self.texture_params['floor_albedo_jitter'] = [jitter_factor * 0.1 * random.uniform(-1, 1) for _ in range(3)] + [0.0]
        self.texture_params['object_albedo_jitter'] = [jitter_factor * 0.15 * random.uniform(-1, 1) for _ in range(3)] + [0.0]

    def apply_physics_randomization(self):
        """Apply physics randomization"""
        if not self.physics_enabled:
            return

        # Add random variations to physics parameters
        base_friction_static = 0.3 + 0.6 * self.severity_initial
        variation = 0.1 * self.current_severity * random.uniform(-1, 1)
        self.physics_params['friction_static'] = max(0.3, min(0.9, base_friction_static + variation))

        base_friction_dynamic = 0.2 + 0.6 * self.severity_initial
        variation = 0.1 * self.current_severity * random.uniform(-1, 1)
        self.physics_params['friction_dynamic'] = max(0.2, min(0.8, base_friction_dynamic + variation))

        base_mass_mult = 0.8 + 0.4 * self.severity_initial
        variation = 0.05 * self.current_severity * random.uniform(-1, 1)
        self.physics_params['mass_multiplier'] = max(0.8, min(1.2, base_mass_mult + variation))

        base_damping_mult = 0.5 + 1.5 * self.severity_initial
        variation = 0.2 * self.current_severity * random.uniform(-1, 1)
        self.physics_params['joint_damping_multiplier'] = max(0.5, min(2.0, base_damping_mult + variation))

        # Publish physics parameters
        physics_msg = String()
        physics_msg.data = json.dumps(self.physics_params)
        self.physics_params_pub.publish(physics_msg)

    def apply_sensor_randomization(self):
        """Apply sensor randomization"""
        if not self.sensor_enabled:
            return

        # Add random variations to sensor parameters
        base_camera_noise = 0.001 + 0.009 * self.severity_initial
        variation = 0.001 * self.current_severity * random.uniform(0, 1)
        self.sensor_params['camera_noise_stddev'] = min(0.01, base_camera_noise + variation)

        base_lidar_noise = 0.001 + 0.019 * self.severity_initial
        variation = 0.002 * self.current_severity * random.uniform(0, 1)
        self.sensor_params['lidar_noise_stddev'] = min(0.02, base_lidar_noise + variation)

        base_imu_noise = 0.001 + 0.019 * self.severity_initial
        variation = 0.002 * self.current_severity * random.uniform(0, 1)
        self.sensor_params['imu_acc_noise_density'] = min(0.02, base_imu_noise + variation)

        base_distortion = -0.1 + 0.2 * self.severity_initial
        variation = 0.05 * self.current_severity * random.uniform(-1, 1)
        self.sensor_params['camera_distortion_k1'] = max(-0.1, min(0.1, base_distortion + variation))
        self.sensor_params['camera_distortion_k2'] = max(-0.1, min(0.1, base_distortion + variation))

        # Publish sensor parameters
        sensor_msg = String()
        sensor_msg.data = json.dumps(self.sensor_params)
        self.sensor_params_pub.publish(sensor_msg)

    def apply_robot_randomization(self):
        """Apply robot-specific randomization"""
        # Add random variations to robot parameters
        base_stiffness_mult = 0.8 + 0.4 * self.severity_initial
        variation = 0.05 * self.current_severity * random.uniform(-1, 1)
        self.robot_params['joint_stiffness_multiplier'] = max(0.8, min(1.2, base_stiffness_mult + variation))

        base_damping_mult = 0.5 + 1.0 * self.severity_initial
        variation = 0.1 * self.current_severity * random.uniform(-1, 1)
        self.robot_params['joint_damping_multiplier'] = max(0.5, min(1.5, base_damping_mult + variation))

        base_kp_mult = 0.8 + 0.4 * self.severity_initial
        variation = 0.05 * self.current_severity * random.uniform(-1, 1)
        self.robot_params['balance_kp_multiplier'] = max(0.8, min(1.2, base_kp_mult + variation))

        base_kd_mult = 0.8 + 0.4 * self.severity_initial
        variation = 0.05 * self.current_severity * random.uniform(-1, 1)
        self.robot_params['balance_kd_multiplier'] = max(0.8, min(1.2, base_kd_mult + variation))

    def update_success_rate(self, success_rate):
        """Update success rate for curriculum learning"""
        with self.randomization_lock:
            self.success_rate_history.append(success_rate)

            # Check if we should advance to the next curriculum stage
            if len(self.success_rate_history) >= 10:  # Need at least 10 samples
                avg_success_rate = sum(self.success_rate_history) / len(self.success_rate_history)

                # Advance curriculum if success rate is high enough
                if avg_success_rate > 0.7 and self.current_severity < self.severity_max * 0.8:
                    self.curriculum_stage = min(2, self.curriculum_stage + 1)  # Max 3 stages (0, 1, 2)
                    self.get_logger().info(f'Curriculum advanced to stage {self.curriculum_stage} with success rate {avg_success_rate:.3f}')

    def get_current_randomization_params(self):
        """Get current randomization parameters"""
        with self.randomization_lock:
            return {
                'severity': self.current_severity,
                'curriculum_stage': self.curriculum_stage,
                'lighting': self.lighting_params.copy(),
                'texture': self.texture_params.copy(),
                'physics': self.physics_params.copy(),
                'sensor': self.sensor_params.copy(),
                'robot': self.robot_params.copy()
            }

    def reset_randomization(self):
        """Reset randomization to initial state"""
        with self.randomization_lock:
            self.current_severity = self.severity_initial
            self.step_count = 0
            self.curriculum_stage = 0
            self.success_rate_history.clear()

            self.initialize_randomization_parameters()

            self.get_logger().info('Domain randomization reset to initial state')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    dr_node = DomainRandomizationNode()

    try:
        rclpy.spin(dr_node)
    except KeyboardInterrupt:
        dr_node.get_logger().info('Domain Randomization Node interrupted by user')
    finally:
        dr_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()