# Quickstart Guide: NVIDIA Isaac AI Robot Brain

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 LTS
   - NVIDIA GPU with compute capability 6.0+ (RTX 3080 or higher recommended)
   - CUDA 11.8+ with compatible driver
   - 16GB+ RAM, 1TB+ free disk space
   - ROS 2 Humble Hawksbill installed

2. **Software Dependencies**:
   ```bash
   # Install NVIDIA Isaac Sim
   # Follow instructions at: https://docs.nvidia.com/isaac/isaac_sim/index.html

   # Install Isaac ROS dependencies
   sudo apt update && sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   sudo apt install ros-humble-isaac-ros-*  # Install all Isaac ROS packages
   ```

3. **Isaac Sim Setup**:
   - Download and install Isaac Sim from NVIDIA Developer website
   - Configure Isaac Sim with proper GPU drivers
   - Verify Isaac Sim can launch and run basic simulations

## Installation

1. **Clone the repository**:
   ```bash
   cd ~/humanoid_ws/src
   git clone [repository-url]  # Replace with actual repository URL
   ```

2. **Install Isaac ROS packages**:
   ```bash
   cd ~/humanoid_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

3. **Verify Isaac Sim integration**:
   - Launch Isaac Sim and verify it can import USD files
   - Test basic simulation functionality
   - Verify ROS 2 bridge connection

## Basic Usage

1. **Launch Isaac Sim Environment**:
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 launch isaac_simulation isaac_sim_bridge.launch.py
   ```

2. **Launch Perception Pipeline** (in separate terminal):
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 launch isaac_robot_perception perception_pipeline.launch.py
   ```

3. **Launch Navigation System** (in separate terminal):
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 launch isaac_robot_navigation navigation.launch.py
   ```

4. **Test VSLAM**:
   ```bash
   # Navigate in the simulated environment
   ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."  # Add appropriate goal pose
   ```

## Week 8: Isaac Sim Setup

1. **Verify Isaac Sim Installation**:
   - Launch Isaac Sim application
   - Load a sample USD scene
   - Verify rendering and physics work properly

2. **Test Isaac Sim-ROS Bridge**:
   - Launch the bridge node
   - Verify ROS 2 topics are being published
   - Check robot state visualization in RViz

## Week 9: Perception & Localization

1. **Configure Perception Pipeline**:
   - Set up camera, LiDAR, and IMU processing nodes
   - Calibrate sensor fusion parameters
   - Test real-time processing performance

2. **Run VSLAM**:
   - Launch VSLAM nodes
   - Navigate the robot to create a map
   - Verify localization accuracy

## Week 10: Navigation & AI Training

1. **Configure Nav2 Integration**:
   - Set up costmap parameters
   - Configure path planner for humanoid robot
   - Test obstacle avoidance

2. **Run AI Training**:
   - Generate synthetic datasets in Isaac Sim
   - Train navigation policy
   - Test sim-to-real transfer

## Verification

1. **Check perception pipeline**:
   ```bash
   ros2 topic list | grep -E "(camera|laser|imu)"
   ros2 topic echo /perception/objects
   ```

2. **Verify VSLAM operation**:
   ```bash
   ros2 topic echo /vslam/map
   ros2 topic echo /vslam/pose
   ```

3. **Test navigation**:
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "..."
   ```

## Troubleshooting

- **Isaac Sim won't launch**: Verify GPU drivers and CUDA installation
- **Perception pipeline slow**: Check GPU utilization and Isaac ROS configuration
- **VSLAM drift**: Verify sensor calibration and feature tracking
- **Navigation fails**: Check costmap configuration and obstacle detection
- **AI training poor performance**: Review synthetic dataset quality and domain randomization settings