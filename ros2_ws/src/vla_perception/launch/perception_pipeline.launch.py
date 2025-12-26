from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_vla_perception = get_package_share_directory('vla_perception')

    # Load configuration files
    object_detection_config = os.path.join(pkg_vla_perception, 'config', 'object_detection.yaml')
    scene_understanding_config = os.path.join(pkg_vla_perception, 'config', 'scene_understanding.yaml')

    return LaunchDescription([
        # Object detection node
        Node(
            package='vla_perception',
            executable='object_detection_node',
            name='object_detection_node',
            parameters=[object_detection_config],
            output='screen'
        ),

        # Scene analysis node
        Node(
            package='vla_perception',
            executable='scene_analysis_node',
            name='scene_analysis_node',
            parameters=[scene_understanding_config],
            output='screen'
        ),

        # Visual grounding node
        Node(
            package='vla_perception',
            executable='visual_grounding_node',
            name='visual_grounding_node',
            output='screen'
        ),
    ])