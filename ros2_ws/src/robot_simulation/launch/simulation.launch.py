from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_simulation')
    robot_pkg_share = get_package_share_directory('robot_description')

    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'simple_room.world'),
        description='Path to the world file to load'
    )

    # Launch Gazebo with the world file
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        arguments=[LaunchConfiguration('world'), '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(robot_pkg_share, 'urdf', 'humanoid.urdf')).read()
        }]
    )

    return LaunchDescription([
        world_file_arg,
        gazebo_node,
        robot_state_publisher_node
    ])