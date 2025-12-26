#!/usr/bin/env python3

"""
Simulation Controller Node for Gazebo-Unity synchronization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import time

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Publisher for Unity visualization
        self.unity_pub = self.create_publisher(JointState, '/unity_visualization/robot_state', 10)

        # Subscriber for joint states from Gazebo
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for simulation control commands
        self.sim_control_pub = self.create_publisher(String, '/simulation_control', 10)

        # Timer to synchronize data to Unity
        self.timer = self.create_timer(0.033, self.sync_to_unity)  # ~30 FPS for Unity

        self.last_joint_state = None
        self.get_logger().info('Simulation Controller Node Started')

    def joint_state_callback(self, msg):
        """Callback for joint state messages from Gazebo"""
        self.last_joint_state = msg

    def sync_to_unity(self):
        """Synchronize joint states to Unity visualization"""
        if self.last_joint_state is not None:
            # Publish joint states to Unity
            unity_msg = JointState()
            unity_msg.header.stamp = self.get_clock().now().to_msg()
            unity_msg.header.frame_id = 'unity_frame'
            unity_msg.name = self.last_joint_state.name
            unity_msg.position = self.last_joint_state.position
            unity_msg.velocity = self.last_joint_state.velocity
            unity_msg.effort = self.last_joint_state.effort

            self.unity_pub.publish(unity_msg)

    def start_simulation(self):
        """Send command to start simulation"""
        cmd_msg = String()
        cmd_msg.data = 'start'
        self.sim_control_pub.publish(cmd_msg)

    def stop_simulation(self):
        """Send command to stop simulation"""
        cmd_msg = String()
        cmd_msg.data = 'stop'
        self.sim_control_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    sim_controller = SimulationController()

    try:
        rclpy.spin(sim_controller)
    except KeyboardInterrupt:
        sim_controller.get_logger().info('Simulation Controller interrupted by user')
    finally:
        sim_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()