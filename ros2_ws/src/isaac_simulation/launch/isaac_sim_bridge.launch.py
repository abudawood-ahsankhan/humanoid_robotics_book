import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('isaac_simulation')

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot'
    )

    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='60.0',
        description='Publish frequency in Hz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    publish_frequency = LaunchConfiguration('publish_frequency')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Isaac Sim bridge node
    isaac_sim_bridge_node = Node(
        package='isaac_simulation',
        executable='isaac_sim_bridge_node',  # This will be created when the package is built
        name='isaac_sim_bridge',
        parameters=[
            {'robot_name': robot_name},
            {'publish_frequency': publish_frequency},
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        # In a real implementation, this would be the Python script directly
        # For now, we'll reference it as a script
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Isaac ROS NITROS node for optimized data transfer
    nitros_node = Node(
        package='isaac_ros_nitros',
        executable='isaac_ros_nitros_type_adapter_node',
        name='nitros_type_adapter',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Isaac ROS Image Proc node for camera processing
    image_proc_node = Node(
        package='isaac_ros_image_proc',
        executable='isaac_ros_image_proc_node',
        name='image_proc',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Isaac ROS Visual SLAM node (will be used later in the pipeline)
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'enable_rectified_pose': True},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'camera_frame': 'camera_link'}
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        publish_frequency_arg,
        use_sim_time_arg,
        isaac_sim_bridge_node,
        nitros_node,
        image_proc_node,
        visual_slam_node
    ])