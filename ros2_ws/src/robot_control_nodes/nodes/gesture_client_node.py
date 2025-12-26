#!/usr/bin/env python3

"""
Gesture Service Client Node for ROS 2 Robotics Module
This node provides a client to call the gesture service
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from robot_control_nodes.srv import GestureTrigger


class GestureClientNode(Node):
    def __init__(self):
        super().__init__('gesture_client_node')

        # Create client for the gesture service
        self.cli = self.create_client(GestureTrigger, 'trigger_gesture')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gesture service not available, waiting again...')

        self.get_logger().info('Gesture Client Node has been started')

    def trigger_gesture(self, gesture_name):
        """Call the gesture service to execute a specific gesture"""
        request = GestureTrigger.Request()
        request.gesture_name = gesture_name

        # Make the service call
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully triggered gesture: {gesture_name}')
            else:
                self.get_logger().error(f'Failed to trigger gesture {gesture_name}: {response.message}')
            return response
        else:
            self.get_logger().error(f'Exception while calling service: {future.exception()}')
            return None


def main(args=None):
    rclpy.init(args=args)

    gesture_client_node = GestureClientNode()

    # Example: trigger a wave gesture
    gesture_client_node.get_logger().info('Triggering wave gesture...')
    response = gesture_client_node.trigger_gesture('wave')

    if response:
        gesture_client_node.get_logger().info(f'Service response: {response.message}')

    # Clean shutdown
    gesture_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()