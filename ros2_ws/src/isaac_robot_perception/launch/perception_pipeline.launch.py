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
            'perception_pipeline_config.yaml'
        ]),
        description='Path to the perception pipeline configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    enable_image_proc_arg = DeclareLaunchArgument(
        'enable_image_proc',
        default_value='true',
        description='Enable image processing components'
    )

    enable_object_detection_arg = DeclareLaunchArgument(
        'enable_object_detection',
        default_value='true',
        description='Enable object detection components'
    )

    enable_segmentation_arg = DeclareLaunchArgument(
        'enable_segmentation',
        default_value='true',
        description='Enable semantic segmentation components'
    )

    enable_vslam_arg = DeclareLaunchArgument(
        'enable_vslam',
        default_value='true',
        description='Enable visual SLAM components'
    )

    enable_sensor_fusion_arg = DeclareLaunchArgument(
        'enable_sensor_fusion',
        default_value='true',
        description='Enable sensor fusion components'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_image_proc = LaunchConfiguration('enable_image_proc')
    enable_object_detection = LaunchConfiguration('enable_object_detection')
    enable_segmentation = LaunchConfiguration('enable_segmentation')
    enable_vslam = LaunchConfiguration('enable_vslam')
    enable_sensor_fusion = LaunchConfiguration('enable_sensor_fusion')

    # Isaac ROS Image Proc components
    image_proc_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_image_proc),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='image_rectify_node',
                parameters=[config_file],
                remappings=[
                    ('image_raw', '/sensors/camera/image_raw'),
                    ('camera_info', '/sensors/camera/camera_info'),
                    ('image_rect', '/sensors/camera/image_rect_color')
                ]
            ),
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='image_resize_node',
                parameters=[config_file],
                remappings=[
                    ('image', '/sensors/camera/image_rect_color'),
                    ('camera_info', '/sensors/camera/camera_info'),
                    ('resized/image', '/sensors/camera/image_resized'),
                    ('resized/camera_info', '/sensors/camera/camera_info_resized')
                ]
            )
        ]
    )

    # Isaac ROS Object Detection components
    detectnet_container = ComposableNodeContainer(
        name='detectnet_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_object_detection),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
                name='detectnet_node',
                parameters=[
                    config_file,
                    {'model_name': 'detectnet_coco'},
                    {'input_width': 640},
                    {'input_height': 480},
                    {'confidence_threshold': 0.7},
                    {'max_detections': 100},
                    {'use_padding': True},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('image', '/sensors/camera/image_resized'),
                    ('camera_info', '/sensors/camera/camera_info_resized'),
                    ('detections', '/perception/objects'),
                    ('mask', '/perception/detection_masks')
                ]
            )
        ]
    )

    # Isaac ROS Semantic Segmentation components
    segmentation_container = ComposableNodeContainer(
        name='segmentation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_segmentation),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_segmentation',
                plugin='nvidia::isaac_ros::segmentation::SegmentationNode',
                name='segmentation_node',
                parameters=[
                    config_file,
                    {'model_name': 'segway_coco'},
                    {'input_width': 640},
                    {'input_height': 480},
                    {'confidence_threshold': 0.5},
                    {'use_nitros_type_adaptation': True}
                ],
                remappings=[
                    ('image', '/sensors/camera/image_resized'),
                    ('segmentation', '/perception/segmentation'),
                    ('colormap', '/perception/segmentation_colormap')
                ]
            )
        ]
    )

    # Isaac ROS Visual SLAM components
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_vslam),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_node',
                parameters=[
                    config_file,
                    {'use_sim_time': use_sim_time},
                    {'enable_rectified_pose': True},
                    {'map_frame': 'map'},
                    {'odom_frame': 'odom'},
                    {'base_frame': 'base_link'},
                    {'camera_frame': 'camera_link'},
                    {'enable_occupancy_map': True},
                    {'occupancy_map_resolution': 0.05},
                    {'occupancy_map_size_x': 20.0},
                    {'occupancy_map_size_y': 20.0}
                ],
                remappings=[
                    ('visual_slam/image', '/sensors/camera/image_rect_color'),
                    ('visual_slam/camera_info', '/sensors/camera/camera_info'),
                    ('visual_slam/imu', '/sensors/imu/data'),
                    ('visual_slam/pose', '/vslam/pose'),
                    ('visual_slam/twist', '/vslam/twist'),
                    ('visual_slam/feature0', '/vslam/features'),
                    ('visual_slam/map', '/vslam/map'),
                    ('visual_slam/path', '/vslam/path_optimized')
                ]
            )
        ]
    )

    # Isaac ROS Sensor Fusion components
    sensor_fusion_container = ComposableNodeContainer(
        name='sensor_fusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_sensor_fusion),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::PointCloudConcatenate',
                name='pointcloud_concatenate_node',
                parameters=[config_file],
                remappings=[
                    ('cloud_1', '/sensors/lidar/points'),
                    ('cloud_2', '/sensors/depth/points'),
                    ('concatenated_cloud', '/sensors/pointcloud_concatenated')
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
                    ('topic_carrier', '/sensors/camera/image_raw_nitros'),
                    ('topic_original', '/sensors/camera/image_raw')
                ]
            )
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        enable_image_proc_arg,
        enable_object_detection_arg,
        enable_segmentation_arg,
        enable_vslam_arg,
        enable_sensor_fusion_arg,

        # Launch containers
        image_proc_container,
        detectnet_container,
        segmentation_container,
        visual_slam_container,
        sensor_fusion_container,
        nitros_container
    ])