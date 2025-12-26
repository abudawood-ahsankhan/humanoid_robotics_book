import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='300.0',
        description='Duration of the sim-to-real transfer test in seconds'
    )

    transfer_success_threshold_arg = DeclareLaunchArgument(
        'transfer_success_threshold',
        default_value='0.7',
        description='Minimum success rate threshold for transfer validation'
    )

    randomization_severity_arg = DeclareLaunchArgument(
        'randomization_severity',
        default_value='0.5',
        description='Severity level for domain randomization (0.0 to 1.0)'
    )

    enable_domain_randomization_arg = DeclareLaunchArgument(
        'enable_domain_randomization',
        default_value='True',
        description='Enable domain randomization during training'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    # Get launch configurations
    test_duration = LaunchConfiguration('test_duration')
    transfer_success_threshold = LaunchConfiguration('transfer_success_threshold')
    randomization_severity = LaunchConfiguration('randomization_severity')
    enable_domain_randomization = LaunchConfiguration('enable_domain_randomization')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get package shares
    isaac_robot_ai_pkg = FindPackageShare('isaac_robot_ai')
    isaac_simulation_pkg = FindPackageShare('isaac_simulation')
    isaac_robot_perception_pkg = FindPackageShare('isaac_robot_perception')

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

    # Domain randomization node
    domain_randomization_node = Node(
        package='isaac_robot_ai',
        executable='domain_randomization_node',
        name='domain_randomization_node',
        parameters=[
            PathJoinSubstitution([
                isaac_robot_ai_pkg,
                'config',
                'domain_randomization_config.yaml'
            ]),
            {'use_sim_time': use_sim_time},
            {'severity_initial': randomization_severity},
            {'severity_max': 0.8},
            {'severity_growth_rate': 0.0001}
        ],
        condition=IfCondition(enable_domain_randomization),
        output='screen'
    )

    # Navigation policy training node
    navigation_policy_node = Node(
        package='isaac_robot_ai',
        executable='navigation_policy_training_node',
        name='navigation_policy_training_node',
        parameters=[
            PathJoinSubstitution([
                isaac_robot_ai_pkg,
                'config',
                'navigation_policy_config.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Sim-to-real transfer test node
    sim_to_real_test_node = Node(
        package='isaac_robot_ai',
        executable='test_sim_to_real_transfer',
        name='sim_to_real_transfer_test',
        parameters=[
            {'test_duration': test_duration},
            {'transfer_success_threshold': transfer_success_threshold},
            {'domain_randomization_enabled': enable_domain_randomization},
            {'randomization_severity': randomization_severity},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # RViz visualization (optional)
    rviz_config = PathJoinSubstitution([
        isaac_robot_ai_pkg,
        'rviz',
        'sim_to_real_transfer_test.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sim_to_real',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_rviz', default='False'))
    )

    return LaunchDescription([
        test_duration_arg,
        transfer_success_threshold_arg,
        randomization_severity_arg,
        enable_domain_randomization_arg,
        use_sim_time_arg,

        # Launch Isaac Sim environment
        isaac_sim_launch,

        # Launch perception pipeline
        perception_pipeline_launch,

        # Launch domain randomization (if enabled)
        domain_randomization_node,

        # Launch navigation policy training
        navigation_policy_node,

        # Launch sim-to-real transfer test
        sim_to_real_test_node,

        # Launch RViz for visualization (optional)
        rviz_node
    ])