#!/usr/bin/env python3

"""
Joint Subscriber Node for ROS 2 Robotics Module
This node subscribes to joint commands and updates joint positions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from robot_control_nodes.msg import JointCommand
import math


class JointSubscriberNode(Node):
    def __init__(self):
        super().__init__('joint_subscriber_node')

        # Create subscriber for joint commands
        self.subscription = self.create_subscription(
            JointCommand,
            'joint_command',
            self.joint_command_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create publisher for updated joint states
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

        self.get_logger().info('Joint Subscriber Node has been started')

    def joint_command_callback(self, msg):
        """Callback to handle incoming joint commands"""
        try:
            # Find the index of the joint to update
            if msg.joint_name in self.joint_names:
                idx = self.joint_names.index(msg.joint_name)

                # Update joint position with the target position
                self.joint_positions[idx] = msg.target_position
                self.joint_velocities[idx] = msg.target_velocity
                self.joint_efforts[idx] = msg.effort

                self.get_logger().info(
                    f'Updated joint {msg.joint_name}: pos={msg.target_position}, '
                    f'vel={msg.target_velocity}, effort={msg.effort}'
                )
            else:
                self.get_logger().warn(f'Unknown joint name: {msg.joint_name}')
        except Exception as e:
            self.get_logger().error(f'Error processing joint command: {str(e)}')

    def timer_callback(self):
        """Publish current joint states"""
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

    joint_subscriber_node = JointSubscriberNode()

    try:
        rclpy.spin(joint_subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()