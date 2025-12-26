from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_vla_execution = get_package_share_directory('vla_execution')

    # Load configuration files
    navigation_config = os.path.join(pkg_vla_execution, 'config', 'navigation_config.yaml')
    manipulation_config = os.path.join(pkg_vla_execution, 'config', 'manipulation_config.yaml')
    safety_config = os.path.join(pkg_vla_execution, 'config', 'safety_config.yaml')

    return LaunchDescription([
        # Execution node
        Node(
            package='vla_execution',
            executable='execution_node',
            name='execution_node',
            parameters=[navigation_config, manipulation_config, safety_config],
            output='screen'
        ),

        # Navigation node
        Node(
            package='vla_execution',
            executable='navigation_node',
            name='navigation_node',
            parameters=[navigation_config],
            output='screen'
        ),

        # Manipulation node
        Node(
            package='vla_execution',
            executable='manipulation_node',
            name='manipulation_node',
            parameters=[manipulation_config],
            output='screen'
        ),

        # Action sequencer node
        Node(
            package='vla_execution',
            executable='action_sequencer_node',
            name='action_sequencer_node',
            parameters=[safety_config],
            output='screen'
        ),
    ])