from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_vla_integration = get_package_share_directory('vla_integration')

    # Load configuration files
    voice_config = os.path.join(pkg_vla_integration, 'config', 'voice_config.yaml')
    execution_config = os.path.join(pkg_vla_integration, 'config', 'execution_config.yaml')
    perception_config = os.path.join(pkg_vla_integration, 'config', 'perception_config.yaml')

    return LaunchDescription([
        # Voice command node
        Node(
            package='vla_integration',
            executable='voice_command_node',
            name='voice_command_node',
            parameters=[voice_config],
            output='screen'
        ),

        # VLA coordinator node
        Node(
            package='vla_integration',
            executable='vla_coordinator_node',
            name='vla_coordinator_node',
            parameters=[execution_config, perception_config],
            output='screen'
        ),

        # Safety monitor node
        Node(
            package='vla_integration',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen'
        ),
    ])