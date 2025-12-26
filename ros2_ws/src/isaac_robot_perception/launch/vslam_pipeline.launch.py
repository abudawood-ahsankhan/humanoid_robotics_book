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
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_robot_perception'),
            'config',
            'vslam_config.yaml'
        ]),
        description='Path to the VSLAM configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    enable_feature_init_arg = DeclareLaunchArgument(
        'enable_feature_init',
        default_value='true',
        description='Enable feature initialization components'
    )

    enable_feature_tracking_arg = DeclareLaunchArgument(
        'enable_feature_tracking',
        default_value='true',
        description='Enable feature tracking components'
    )

    enable_visual_odometry_arg = DeclareLaunchArgument(
        'enable_visual_odometry',
        default_value='true',
        description='Enable visual odometry components'
    )

    enable_loop_closure_arg = DeclareLaunchArgument(
        'enable_loop_closure',
        default_value='true',
        description='Enable loop closure detection components'
    )

    enable_pose_optimization_arg = DeclareLaunchArgument(
        'enable_pose_optimization',
        default_value='true',
        description='Enable pose graph optimization components'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_feature_init = LaunchConfiguration('enable_feature_init')
    enable_feature_tracking = LaunchConfiguration('enable_feature_tracking')
    enable_visual_odometry = LaunchConfiguration('enable_visual_odometry')
    enable_loop_closure = LaunchConfiguration('enable_loop_closure')
    enable_pose_optimization = LaunchConfiguration('enable_pose_optimization')

    # Isaac ROS VSLAM Feature Initialization
    feature_init_container = ComposableNodeContainer(
        name='feature_init_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_feature_init),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::FeatureInitializationNode',
                name='feature_init_node',
                parameters=[
                    config_file,
                    {'max_features': 2000},
                    {'min_distance': 5},
                    {'quality_level': 0.01},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('image', '/sensors/camera/image_rect_color'),
                    ('features', '/vslam/features_raw')
                ]
            )
        ]
    )

    # Isaac ROS VSLAM Feature Tracking
    feature_tracking_container = ComposableNodeContainer(
        name='feature_tracking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_feature_tracking),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::FeatureTrackingNode',
                name='feature_tracking_node',
                parameters=[
                    config_file,
                    {'max_features': 2000},
                    {'min_distance': 5},
                    {'quality_level': 0.01},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('image', '/sensors/camera/image_rect_color'),
                    ('features', '/vslam/features_raw'),
                    ('tracked_features', '/vslam/features_tracked')
                ]
            )
        ]
    )

    # Isaac ROS VSLAM Visual Odometry
    visual_odometry_container = ComposableNodeContainer(
        name='visual_odometry_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_visual_odometry),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualOdometryNode',
                name='visual_odometry_node',
                parameters=[
                    config_file,
                    {'max_features': 2000},
                    {'min_distance': 5},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('features_tracked', '/vslam/features_tracked'),
                    ('odometry', '/vslam/odometry_raw'),
                    ('pose', '/vslam/pose_raw')
                ]
            )
        ]
    )

    # Isaac ROS VSLAM Loop Closure
    loop_closure_container = ComposableNodeContainer(
        name='loop_closure_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_loop_closure),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::LoopClosureNode',
                name='loop_closure_node',
                parameters=[
                    config_file,
                    {'vocab_path': '/opt/nvidia/isaac_ros/isaac_ros_visual_slam/data/intel_vocab.db'},
                    {'min_score': 0.5},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('features_tracked', '/vslam/features_tracked'),
                    ('loop_closure', '/vslam/loop_closure_detected')
                ]
            )
        ]
    )

    # Isaac ROS VSLAM Pose Graph Optimization
    pose_optimization_container = ComposableNodeContainer(
        name='pose_optimization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_pose_optimization),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::PoseGraphOptimizationNode',
                name='pose_optimization_node',
                parameters=[
                    config_file,
                    {'max_iterations': 100},
                    {'convergence_epsilon': 1e-6},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('loop_closure', '/vslam/loop_closure_detected'),
                    ('pose_optimized', '/vslam/pose_optimized'),
                    ('path_optimized', '/vslam/path_optimized')
                ]
            )
        ]
    )

    # Isaac ROS VSLAM IMU Integration
    imu_integration_container = ComposableNodeContainer(
        name='imu_integration_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_imu',
                plugin='nvidia::isaac_ros::imu::IMUMultiplexerNode',
                name='imu_multiplexer_node',
                parameters=[config_file],
                remappings=[
                    ('imu_1', '/sensors/imu/data'),
                    ('imu_out', '/vslam/imu_fused')
                ]
            )
        ]
    )

    # Isaac ROS VSLAM Map Generation
    map_generation_container = ComposableNodeContainer(
        name='map_generation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_occupancy_grid',
                plugin='nvidia::isaac_ros::occupancy_grid::OccupancyGridNode',
                name='occupancy_grid_node',
                parameters=[
                    config_file,
                    {'resolution': 0.05},
                    {'size_x': 20.0},
                    {'size_y': 20.0}
                ],
                remappings=[
                    ('pose', '/vslam/pose_optimized'),
                    ('map', '/vslam/map')
                ]
            )
        ]
    )

    # Isaac ROS Nitros components for optimized data transfer
    nitros_container = ComposableNodeContainer(
        name='nitros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::TypeAdapterNode',
                name='nitros_image_type_adapter',
                parameters=[
                    config_file,
                    {'compatible_format': 'nitros_image_rgb8'}
                ],
                remappings=[
                    ('topic_carrier', '/sensors/camera/image_rect_color_nitros'),
                    ('topic_original', '/sensors/camera/image_rect_color')
                ]
            ),
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::TypeAdapterNode',
                name='nitros_imu_type_adapter',
                parameters=[
                    config_file,
                    {'compatible_format': 'nitros_imu'}
                ],
                remappings=[
                    ('topic_carrier', '/sensors/imu/data_nitros'),
                    ('topic_original', '/sensors/imu/data')
                ]
            )
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        enable_feature_init_arg,
        enable_feature_tracking_arg,
        enable_visual_odometry_arg,
        enable_loop_closure_arg,
        enable_pose_optimization_arg,

        # Launch containers
        feature_init_container,
        feature_tracking_container,
        visual_odometry_container,
        loop_closure_container,
        pose_optimization_container,
        imu_integration_container,
        map_generation_container,
        nitros_container
    ])