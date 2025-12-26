# Quickstart Guide: Gazebo & Unity Digital Twin

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
   - Minimum 8GB RAM, 4+ CPU cores
   - NVIDIA GPU with Unity-compatible drivers (for Unity visualization)
   - 10GB free disk space

2. **Software Dependencies**:
   ```bash
   # Install ROS 2 Humble Hawksbill
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop ros-humble-ros-base
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
   ```

   ```bash
   # Install Unity Hub and Unity Editor (2021.3 LTS or later)
   # Download from: https://unity.com/download
   # Install with Linux support enabled
   ```

3. **Workspace Setup**:
   ```bash
   # Create and setup ROS 2 workspace
   mkdir -p ~/humanoid_ws/src
   cd ~/humanoid_ws
   colcon build
   source install/setup.bash
   ```

## Installation

1. **Clone the repository**:
   ```bash
   cd ~/humanoid_ws/src
   git clone [repository-url]  # Replace with actual repository URL
   ```

2. **Install additional dependencies**:
   ```bash
   cd ~/humanoid_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

3. **Setup Unity Project**:
   - Open Unity Hub
   - Import the Unity project from `~/humanoid_ws/src/robot_visualization`
   - Install Unity Robotics Hub package via Package Manager
   - Install ROS# package for ROS 2 communication

## Basic Usage

1. **Launch Gazebo Simulation**:
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 launch robot_simulation simulation.launch.py
   ```

2. **Launch Unity Visualization** (in separate terminal):
   ```bash
   # Navigate to Unity project and run
   # Or open in Unity Editor and press Play
   ```

3. **Launch Robot Control Nodes** (in separate terminal):
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 launch robot_control_nodes robot_control.launch.py
   ```

4. **Test Joint Control**:
   ```bash
   # Send joint position commands
   ros2 topic pub /joint_commands trajectory_msgs/msg/JointTrajectory "..."
   ```

## Verification

1. **Check simulation is running**:
   ```bash
   ros2 topic list | grep -E "(joint|sensor)"
   ```

2. **Verify sensor data**:
   ```bash
   ros2 topic echo /sensor_data/laser_scan
   ros2 topic echo /sensor_data/imu
   ros2 topic echo /sensor_data/depth_camera
   ```

3. **Check Unity connection**:
   - Unity visualization should reflect robot movements in Gazebo
   - Joint positions should synchronize between environments

## Troubleshooting

- **Gazebo won't start**: Ensure NVIDIA drivers are properly installed if using GPU acceleration
- **Unity connection fails**: Check that ROS 2 bridge is properly configured in Unity project
- **Sensor data not publishing**: Verify Gazebo plugins are correctly configured in robot model
- **Joint control not working**: Check that joint names match between URDF and control nodes