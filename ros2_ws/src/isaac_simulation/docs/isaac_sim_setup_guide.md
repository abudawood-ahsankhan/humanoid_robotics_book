# Isaac Sim Setup and Verification Guide

## Overview

This guide provides comprehensive instructions for setting up NVIDIA Isaac Sim with the humanoid robot for AI-driven perception, navigation, and decision-making. The setup includes installation, configuration, and verification steps to ensure the system is ready for synthetic data generation and AI training.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Installation](#installation)
3. [Configuration](#configuration)
4. [Verification](#verification)
5. [Troubleshooting](#troubleshooting)
6. [Next Steps](#next-steps)

## System Requirements

### Hardware Requirements
- **CPU**: Intel i7 or AMD Ryzen 7 (8+ cores recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA RTX 3080 or higher with 10GB+ VRAM
- **Storage**: 1TB SSD minimum (500GB for Isaac Sim + 500GB for datasets)
- **OS**: Ubuntu 22.04 LTS

### Software Requirements
- **CUDA**: 11.8 or higher with compatible drivers
- **ROS 2**: Humble Hawksbill
- **Isaac Sim**: 2023.1 or higher
- **Isaac ROS**: 3.0 or higher

## Installation

### 1. Install NVIDIA GPU Drivers and CUDA

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install NVIDIA drivers
sudo apt install nvidia-driver-535

# Install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-11-8
```

### 2. Install ROS 2 Humble

```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Install Isaac ROS Packages

```bash
# Install Isaac ROS meta-package
sudo apt update
sudo apt install ros-humble-isaac-ros-perception
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-nitros
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-compression
sudo apt install ros-humble-isaac-ros-image-ros-bridge
sudo apt install ros-humble-isaac-ros-gxf
sudo apt install ros-humble-isaac-ros-build-interfaces
sudo apt install ros-humble-isaac-ros-launch-interfaces
```

### 4. Install Isaac Sim

1. **Sign up for NVIDIA Developer Account**:
   - Visit https://developer.nvidia.com/isaac-sim
   - Create or sign in to your NVIDIA Developer account

2. **Download Isaac Sim**:
   - Download Isaac Sim from the NVIDIA Developer website
   - Choose the appropriate version (recommended: latest stable)

3. **Install Isaac Sim**:
   - Extract the downloaded archive
   - Follow the installation instructions provided with the package
   - Make sure to install all required dependencies

4. **Verify Installation**:
   - Launch Isaac Sim application
   - Check that the application starts without errors
   - Verify rendering and physics work properly

## Configuration

### 1. Create ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
```

### 2. Clone Isaac AI Robot Brain Packages

```bash
cd ~/humanoid_ws/src

# Clone the Isaac AI Robot Brain packages
git clone [repository-url]/isaac_robot_perception.git
git clone [repository-url]/isaac_robot_navigation.git
git clone [repository-url]/isaac_robot_ai.git
git clone [repository-url]/isaac_simulation.git

# Build the workspace again
cd ~/humanoid_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Configure Isaac Sim-ROS Bridge

1. **Launch Isaac Sim**:
   - Start Isaac Sim application
   - Create a new scene or open an existing one

2. **Configure Bridge Settings**:
   - Go to `Isaac ROS` → `ROS Bridge` in Isaac Sim
   - Set ROS Master URI to `http://localhost:11311`
   - Verify connection status

3. **Configure Topic Mappings**:
   - Use the configuration from `isaac_simulation/config/isaac_sim_bridge_config.yaml`
   - Map Isaac Sim sensors to ROS topics as specified in the interfaces document

### 4. Import Humanoid Robot Model

1. **Import URDF**:
   - Use the URDF file from `isaac_simulation/worlds/humanoid_robot.urdf`
   - Follow the import guide in `isaac_simulation/docs/urdf_to_usd_import_guide.md`

2. **Configure Physics**:
   - Apply physics properties from `isaac_simulation/config/physics_config.yaml`
   - Follow the physics configuration guide in `isaac_simulation/docs/physics_configuration_guide.md`

3. **Configure Rendering**:
   - Apply rendering settings from `isaac_simulation/config/rendering_config.yaml`
   - Follow the rendering guide in `isaac_simulation/docs/photorealistic_rendering_guide.md`

## Verification

### 1. Launch Isaac Sim Bridge

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch Isaac Sim bridge
ros2 launch isaac_simulation isaac_sim_bridge.launch.py
```

### 2. Verify Sensor Data

```bash
# Check available topics
ros2 topic list | grep -E "(sensors|camera|lidar|imu)"

# Monitor camera data
ros2 topic echo /sensors/camera/image_raw --field header.stamp

# Monitor LiDAR data
ros2 topic echo /sensors/lidar/points --field header.stamp

# Monitor IMU data
ros2 topic echo /sensors/imu/data --field header.stamp

# Monitor joint states
ros2 topic echo /isaac_sim/joint_states --field position
```

### 3. Test Robot Spawning

1. **Run Spawning Test**:
   ```bash
   python3 src/isaac_simulation/test/test_robot_spawning.py
   ```

2. **Manual Verification**:
   - Verify the robot appears in Isaac Sim
   - Check that all links are visible and properly connected
   - Confirm rendering looks photorealistic
   - Verify physics simulation is stable

### 4. Test Synthetic Dataset Generation

1. **Run Dataset Generation**:
   ```bash
   ros2 launch isaac_robot_ai synthetic_dataset_generator.launch.py \
     output_dir:=/tmp/test_dataset \
     max_samples:=10 \
     capture_frequency:=1.0
   ```

2. **Verify Dataset**:
   ```bash
   python3 src/isaac_robot_ai/test/test_synthetic_dataset_generation.py
   ```

### 5. Test Perception Pipeline

1. **Launch Perception Nodes**:
   ```bash
   ros2 launch isaac_robot_perception perception_pipeline.launch.py
   ```

2. **Verify Perception Output**:
   ```bash
   ros2 topic echo /perception/objects --field detections
   ```

### 6. Test Navigation System

1. **Launch Navigation**:
   ```bash
   ros2 launch isaac_robot_navigation navigation.launch.py
   ```

2. **Send Navigation Goal**:
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}"
   ```

## Troubleshooting

### Common Issues and Solutions

#### Isaac Sim Won't Launch
- **Issue**: Isaac Sim fails to start
- **Solution**:
  - Verify GPU drivers are properly installed: `nvidia-smi`
  - Check CUDA installation: `nvcc --version`
  - Ensure sufficient VRAM is available

#### No Sensor Data Published
- **Issue**: Sensor topics are not publishing data
- **Solution**:
  - Verify Isaac Sim-ROS bridge is running
  - Check topic mappings in bridge configuration
  - Confirm sensors are properly configured in Isaac Sim

#### Robot Physics Issues
- **Issue**: Robot falls through ground or behaves unrealistically
- **Solution**:
  - Verify rigid body components are added to all links
  - Check physics material properties
  - Ensure proper mass and inertia values

#### Poor Rendering Quality
- **Issue**: Rendering appears low quality or unrealistic
- **Solution**:
  - Enable advanced rendering features in Isaac Sim
  - Verify material assignments are correct
  - Check lighting configuration

#### Performance Problems
- **Issue**: Slow simulation or rendering
- **Solution**:
  - Reduce rendering quality settings temporarily
  - Check system resource usage
  - Ensure GPU drivers are up to date

### Diagnostic Commands

```bash
# Check Isaac Sim processes
ps aux | grep -i "isaac\|omniverse"

# Check ROS 2 nodes
ros2 node list

# Check ROS 2 topics
ros2 topic list

# Check system resources
nvidia-smi
htop

# Check Isaac Sim logs
tail -f ~/.nvidia-omniverse/logs/isaac-sim/app/isaac-sim.log
```

## Next Steps

After successful setup and verification:

1. **Run Full Perception Pipeline**: Test the complete Isaac ROS perception pipeline with hardware acceleration
2. **Implement VSLAM**: Configure and test Visual SLAM for mapping and localization
3. **Set up Navigation**: Configure Nav2 integration for autonomous navigation
4. **Train AI Models**: Use generated synthetic datasets to train perception and navigation models
5. **Test Sim-to-Real**: Validate sim-to-real transfer capabilities with real robot data

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Isaac AI Robot Brain Interface Contracts](../contracts/isaac_ros_interfaces.md)
- [Quickstart Guide](../quickstart.md)