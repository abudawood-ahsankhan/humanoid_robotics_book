cmake_minimum_required(VERSION 3.8)
project(robot_simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# Find Gazebo
find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

# Install Python scripts
install(PROGRAMS
  scripts/simulation_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()