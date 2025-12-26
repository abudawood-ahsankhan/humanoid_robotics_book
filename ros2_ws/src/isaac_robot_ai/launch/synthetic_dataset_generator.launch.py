import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('isaac_robot_ai')

    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/synthetic_dataset',
        description='Output directory for dataset'
    )

    dataset_name_arg = DeclareLaunchArgument(
        'dataset_name',
        default_value='isaac_humanoid_dataset',
        description='Name of the dataset'
    )

    max_samples_arg = DeclareLaunchArgument(
        'max_samples',
        default_value='1000',
        description='Maximum number of samples to capture'
    )

    capture_frequency_arg = DeclareLaunchArgument(
        'capture_frequency',
        default_value='1.0',
        description='Capture frequency in Hz'
    )

    # Get launch configurations
    output_dir = LaunchConfiguration('output_dir')
    dataset_name = LaunchConfiguration('dataset_name')
    max_samples = LaunchConfiguration('max_samples')
    capture_frequency = LaunchConfiguration('capture_frequency')

    # Synthetic dataset generator node
    synthetic_dataset_generator_node = Node(
        package='isaac_robot_ai',
        executable='synthetic_dataset_generator',
        name='synthetic_dataset_generator',
        parameters=[
            {'output_dir': output_dir},
            {'dataset_name': dataset_name},
            {'max_samples': max_samples},
            {'capture_frequency': capture_frequency}
        ],
        output='screen'
    )

    return LaunchDescription([
        output_dir_arg,
        dataset_name_arg,
        max_samples_arg,
        capture_frequency_arg,
        synthetic_dataset_generator_node
    ])