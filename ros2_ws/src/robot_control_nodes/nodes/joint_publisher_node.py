#!/usr/bin/env python3

"""
Joint Publisher Node for ROS 2 Robotics Module
This node publishes joint states for the humanoid robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import random


class JointPublisherNode(Node):
    def __init__(self):
        super().__init__('joint_publisher_node')

        # Create publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish joint states at 50Hz
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize joint names based on our URDF
        self.joint_names = [
            'left_shoulder_pitch', 'left_elbow',
            'right_shoulder_pitch', 'right_elbow',
            'left_hip_pitch', 'left_knee',
            'right_hip_pitch', 'right_knee'
        ]

        # Initialize joint positions (starting at 0)
        self.joint_positions = [0.0] * len(self.joint_names)

        # Initialize joint velocities and efforts
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info('Joint Publisher Node has been started')

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published joint states: {self.joint_positions}')


def main(args=None):
    rclpy.init(args=args)

    joint_publisher_node = JointPublisherNode()

    try:
        rclpy.spin(joint_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()