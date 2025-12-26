from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_control_nodes')

    return LaunchDescription([
        # Gesture Service Node
        Node(
            package='robot_control_nodes',
            executable='gesture_service_node',
            name='gesture_service_node',
            output='screen',
            parameters=[]
        ),

        # Movement Action Node
        Node(
            package='robot_control_nodes',
            executable='movement_action_node',
            name='movement_action_node',
            output='screen',
            parameters=[]
        )
    ])