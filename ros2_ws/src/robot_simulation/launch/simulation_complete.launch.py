from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('robot_simulation')
    robot_pkg_share = get_package_share_directory('robot_description')
    gazebo_ros_pkg_share = get_package_share_directory('gazebo_ros')

    # Launch configuration
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'humanoid_room.world'),
        description='Path to the world file to load'
    )

    # Launch Gazebo with the world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
            'gui': 'true'
        }.items()
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

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_gui': False,
            'rate': 50
        }]
    )

    # Spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Launch the simulation controller
    simulation_controller_node = Node(
        package='robot_simulation',
        executable='simulation_controller',
        name='simulation_controller',
        output='screen',
        parameters=[]
    )

    # Launch RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_simulation'),
        'config',
        'simulation.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=None  # Always run
    )

    # Create launch description
    ld = LaunchDescription([
        world_file_arg,
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        simulation_controller_node,
        rviz_node
    ])

    # Register event handler to spawn robot after Gazebo starts
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[spawn_entity_node]
        )
    ))

    return ld