cmake_minimum_required(VERSION 3.8)
project(isaac_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_world_model REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_util REQUIRED)
find_package(isaac_ros_visual_slam REQUIRED)
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)
find_package(launch_param_builder REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
install(PROGRAMS
  nodes/obstacle_avoidance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()