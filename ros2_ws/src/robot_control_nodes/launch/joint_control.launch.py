from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_control_nodes')

    return LaunchDescription([
        # Joint Publisher Node
        Node(
            package='robot_control_nodes',
            executable='joint_publisher_node',
            name='joint_publisher_node',
            output='screen',
            parameters=[]
        ),

        # Joint Subscriber Node
        Node(
            package='robot_control_nodes',
            executable='joint_subscriber_node',
            name='joint_subscriber_node',
            output='screen',
            parameters=[]
        )
    ])