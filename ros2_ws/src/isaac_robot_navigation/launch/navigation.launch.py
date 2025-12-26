import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('isaac_robot_navigation')

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the controllers'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_path_planner_config.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    default_bt_xml_filename_arg = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
        ),
        description='Full path to the behavior tree xml file to use'
    )

    map_subscribe_transient_local_arg = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='false',
        description='Whether to set the map subscriber QoS to transient local'
    )

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Create the actual parameters file substitution
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    # Create the configuration file
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('navigate_to_pose', 'navigate_to_pose')]
    )

    # Recoveries Server
    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('local_costmap/local_costmap', 'local_costmap'),
            ('local_costmap/clear_entirely_local_costmap', 'clear_entirely_local_costmap')
        ]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('navigate_to_pose', 'navigate_to_pose'),
            ('feedback', 'navigate_to_pose/_action/feedback'),
            ('result', 'navigate_to_pose/_action/result'),
            ('goal', 'navigate_to_pose/_action/goal')
        ]
    )

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
            ('feedback', 'cmd_vel_nav_feedback')
        ]
    )

    # Lifecycle Manager for controllers
    lifecycle_manager_controllers = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controllers',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'velocity_smoother'
            ]}
        ]
    )

    # Local costmap
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('scan', 'scan'),
            ('tf', 'tf'),
            ('tf_static', 'tf_static')
        ]
    )

    # Global costmap
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('scan', 'scan'),
            ('tf', 'tf'),
            ('tf_static', 'tf_static')
        ]
    )

    # Lifecycle Manager for costmaps
    lifecycle_manager_costmaps = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmaps',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'local_costmap',
                'global_costmap'
            ]}
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        default_bt_xml_filename_arg,
        map_subscribe_transient_local_arg,
        controller_server_node,
        planner_server_node,
        recoveries_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        local_costmap_node,
        global_costmap_node,
        lifecycle_manager_controllers,
        lifecycle_manager_costmaps
    ])