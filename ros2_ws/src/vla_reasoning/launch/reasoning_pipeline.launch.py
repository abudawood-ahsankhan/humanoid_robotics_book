from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_vla_reasoning = get_package_share_directory('vla_reasoning')

    # Load configuration files
    llm_config = os.path.join(pkg_vla_reasoning, 'config', 'llm_config.yaml')
    planning_config = os.path.join(pkg_vla_reasoning, 'config', 'planning_config.yaml')
    llm_prompts_dir = os.path.join(pkg_vla_reasoning, 'config', 'llm_prompts')

    return LaunchDescription([
        # LLM interface node
        Node(
            package='vla_reasoning',
            executable='llm_interface_node',
            name='llm_interface_node',
            parameters=[llm_config],
            output='screen'
        ),

        # Reasoning node
        Node(
            package='vla_reasoning',
            executable='reasoning_node',
            name='reasoning_node',
            parameters=[planning_config],
            output='screen'
        ),

        # Plan validator node
        Node(
            package='vla_reasoning',
            executable='plan_validator_node',
            name='plan_validator_node',
            parameters=[planning_config],
            output='screen'
        ),
    ])