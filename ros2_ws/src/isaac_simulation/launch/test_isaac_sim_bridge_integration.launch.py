import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='120.0',
        description='Duration of the integration test in seconds'
    )

    robot_spawn_timeout_arg = DeclareLaunchArgument(
        'robot_spawn_timeout',
        default_value='30.0',
        description='Timeout for robot spawning in seconds'
    )

    sensor_timeout_arg = DeclareLaunchArgument(
        'sensor_timeout',
        default_value='10.0',
        description='Timeout for sensor verification in seconds'
    )

    navigation_timeout_arg = DeclareLaunchArgument(
        'navigation_timeout',
        default_value='60.0',
        description='Timeout for navigation test in seconds'
    )

    navigation_goal_x_arg = DeclareLaunchArgument(
        'navigation_goal_x',
        default_value='5.0',
        description='X coordinate of navigation goal'
    )

    navigation_goal_y_arg = DeclareLaunchArgument(
        'navigation_goal_y',
        default_value='5.0',
        description='Y coordinate of navigation goal'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    # Get launch configurations
    test_duration = LaunchConfiguration('test_duration')
    robot_spawn_timeout = LaunchConfiguration('robot_spawn_timeout')
    sensor_timeout = LaunchConfiguration('sensor_timeout')
    navigation_timeout = LaunchConfiguration('navigation_timeout')
    navigation_goal_x = LaunchConfiguration('navigation_goal_x')
    navigation_goal_y = LaunchConfiguration('navigation_goal_y')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package names
    isaac_simulation_pkg = FindPackageShare('isaac_simulation')
    isaac_robot_perception_pkg = FindPackageShare('isaac_robot_perception')
    isaac_robot_navigation_pkg = FindPackageShare('isaac_robot_navigation')

    # Isaac Sim launch
    isaac_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                isaac_simulation_pkg,
                'launch',
                'isaac_sim_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Isaac ROS perception pipeline launch
    perception_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                isaac_robot_perception_pkg,
                'launch',
                'perception_pipeline.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Isaac ROS navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                isaac_robot_navigation_pkg,
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Isaac Sim bridge integration test node
    bridge_integration_test_node = Node(
        package='isaac_simulation',
        executable='isaac_sim_bridge_integration_test',
        name='isaac_sim_bridge_integration_test',
        parameters=[
            {'test_duration': test_duration},
            {'robot_spawn_timeout': robot_spawn_timeout},
            {'sensor_timeout': sensor_timeout},
            {'navigation_timeout': navigation_timeout},
            {'navigation_goal_x': navigation_goal_x},
            {'navigation_goal_y': navigation_goal_y},
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        respawn=False
    )

    # Delay the test node until other systems are initialized
    delayed_test_node = TimerAction(
        period=10.0,  # Wait 10 seconds for systems to initialize
        actions=[bridge_integration_test_node]
    )

    return LaunchDescription([
        test_duration_arg,
        robot_spawn_timeout_arg,
        sensor_timeout_arg,
        navigation_timeout_arg,
        navigation_goal_x_arg,
        navigation_goal_y_arg,
        use_sim_time_arg,

        # Launch Isaac Sim environment
        isaac_sim_launch,

        # Launch perception pipeline
        perception_pipeline_launch,

        # Launch navigation system
        navigation_launch,

        # Launch bridge integration test after delay
        delayed_test_node
    ])