#!/usr/bin/env python3

"""
Gesture Service Server Node for ROS 2 Robotics Module
This node provides services for triggering predefined gestures
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
import time

from robot_control_nodes.srv import GestureTrigger
from robot_control_nodes.action import MoveRobot
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


class GestureServiceNode(Node):
    def __init__(self):
        super().__init__('gesture_service_node')

        # Create service for triggering gestures
        self.srv = self.create_service(
            GestureTrigger,
            'trigger_gesture',
            self.trigger_gesture_callback
        )

        # Publisher for joint commands to control the robot
        self.joint_cmd_publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )

        # Store predefined gestures
        self.gestures = {
            'wave': self.execute_wave_gesture,
            'nod': self.execute_nod_gesture,
            'point': self.execute_point_gesture,
            'dance': self.execute_dance_gesture
        }

        self.get_logger().info('Gesture Service Node has been started')

    def trigger_gesture_callback(self, request, response):
        """Callback to handle gesture trigger requests"""
        gesture_name = request.gesture_name.lower()

        if gesture_name in self.gestures:
            self.get_logger().info(f'Executing gesture: {gesture_name}')
            try:
                # Execute the gesture
                self.gestures[gesture_name]()

                response.success = True
                response.message = f'Gesture {gesture_name} completed successfully'
                self.get_logger().info(f'Gesture {gesture_name} completed')
            except Exception as e:
                response.success = False
                response.message = f'Error executing gesture {gesture_name}: {str(e)}'
                self.get_logger().error(f'Error in gesture {gesture_name}: {str(e)}')
        else:
            response.success = False
            response.message = f'Unknown gesture: {gesture_name}. Available: {list(self.gestures.keys())}'
            self.get_logger().warn(f'Unknown gesture requested: {gesture_name}')

        return response

    def execute_wave_gesture(self):
        """Execute a waving gesture with the right arm"""
        self.get_logger().info('Executing wave gesture')

        # Wave with right arm
        joint_cmd = JointState()
        joint_cmd.name = ['right_shoulder_pitch', 'right_elbow']

        # Position 1: arm up
        joint_cmd.position = [0.5, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(1.0)

        # Position 2: wave
        joint_cmd.position = [0.5, 0.5]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Position 3: wave back
        joint_cmd.position = [0.5, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Position 4: wave again
        joint_cmd.position = [0.5, 0.5]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Return to neutral
        joint_cmd.position = [0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

    def execute_nod_gesture(self):
        """Execute a nodding gesture with the head (simulated with torso)"""
        self.get_logger().info('Executing nod gesture')

        # Nod with head (simulated with torso pitch)
        joint_cmd = JointState()
        joint_cmd.name = ['left_shoulder_pitch', 'right_shoulder_pitch']

        # Nod down
        joint_cmd.position = [-0.3, -0.3]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Nod up
        joint_cmd.position = [0.3, 0.3]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Nod down again
        joint_cmd.position = [-0.3, -0.3]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Return to neutral
        joint_cmd.position = [0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

    def execute_point_gesture(self):
        """Execute a pointing gesture with the right arm"""
        self.get_logger().info('Executing point gesture')

        # Point forward with right arm
        joint_cmd = JointState()
        joint_cmd.name = ['right_shoulder_pitch', 'right_elbow']

        # Position arm to point forward
        joint_cmd.position = [0.0, 1.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(1.0)

        # Hold position
        time.sleep(1.0)

        # Return to neutral
        joint_cmd.position = [0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

    def execute_dance_gesture(self):
        """Execute a simple dance sequence"""
        self.get_logger().info('Executing dance gesture')

        # Simple dance sequence
        joint_cmd = JointState()
        joint_cmd.name = ['left_shoulder_pitch', 'right_shoulder_pitch',
                         'left_hip_pitch', 'right_hip_pitch']

        # Sequence 1: arms up
        joint_cmd.position = [0.5, 0.5, 0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Sequence 2: arms down, legs up
        joint_cmd.position = [0.0, 0.0, 0.3, 0.3]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Sequence 3: arms up, legs down
        joint_cmd.position = [0.5, 0.5, 0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)

        # Sequence 4: back to neutral
        joint_cmd.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_cmd_publisher.publish(joint_cmd)
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    gesture_service_node = GestureServiceNode()

    try:
        rclpy.spin(gesture_service_node)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()