cmake_minimum_required(VERSION 3.8)
project(isaac_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(isaac_ros_visual_slam REQUIRED)
find_package(isaac_ros_image_proc REQUIRED)
find_package(isaac_ros_compression REQUIRED)
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

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

# Install Python scripts
install(PROGRAMS
  nodes/camera_processing_node.py
  nodes/lidar_processing_node.py
  nodes/sensor_fusion_node.py
  nodes/object_detection_pipeline.py
  nodes/semantic_segmentation_pipeline.py
  nodes/advanced_sensor_fusion_node.py
  nodes/visual_slam_node.py
  nodes/feature_tracking_node.py
  nodes/loop_closure_node.py
  nodes/pose_graph_optimization_node.py
  nodes/vslam_pipeline.py
  test/test_vslam_pipeline_with_isaac_sim.py
  test/test_vslam_performance_isaac_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()