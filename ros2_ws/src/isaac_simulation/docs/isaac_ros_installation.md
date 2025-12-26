# Isaac ROS Installation Guide

## Prerequisites

Before installing Isaac ROS packages, ensure you have:

- ROS 2 Humble Hawksbill installed
- NVIDIA GPU with compute capability 6.0+ (RTX 3080 or higher recommended)
- CUDA 11.8+ with compatible driver
- Isaac Sim installed (or at least the Isaac ROS dependencies)

## Installation Steps

1. **Update APT package index**
   ```bash
   sudo apt update
   ```

2. **Install Isaac ROS packages**
   ```bash
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

3. **Alternative: Install all Isaac ROS packages at once**
   ```bash
   sudo apt install ros-humble-isaac-ros-*
   ```

4. **Verify Installation**
   ```bash
   # Check if Isaac ROS packages are available
   ros2 pkg list | grep isaac
   ```

## Hardware Acceleration Verification

1. **Check GPU Utilization**
   ```bash
   nvidia-smi
   ```

2. **Test Isaac ROS Perception Pipeline**
   ```bash
   # Launch a simple perception pipeline to verify hardware acceleration
   ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py
   ```

3. **Monitor GPU Usage During Execution**
   ```bash
   # In another terminal, monitor GPU usage
   watch -n 1 nvidia-smi
   ```

4. **Check Isaac ROS Node Performance**
   ```bash
   # Verify nodes are running and processing data
   ros2 node list
   ros2 topic list
   ros2 topic hz /camera/image_raw  # or other relevant topics
   ```

## Docker Installation (Alternative Method)

If you prefer using Docker for Isaac ROS:

1. **Install Docker and NVIDIA Container Toolkit**
   ```bash
   # Follow instructions at https://docs.docker.com/engine/install/ubuntu/
   # Follow instructions at https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
   ```

2. **Pull Isaac ROS Docker Images**
   ```bash
   docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_visual_slam:latest
   docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_perception:latest
   ```

## Verification Commands

To verify Isaac ROS installation and hardware acceleration:

```bash
# List all Isaac ROS packages
ros2 pkg list | grep isaac

# Check available Isaac ROS launch files
find /opt/ros/humble/share/ -name "*isaac*launch*" -type f

# Verify Isaac ROS nodes can be launched
ros2 run --list | grep isaac

# Test hardware acceleration with a simple perception node
ros2 launch isaac_ros_image_proc image_proc.launch.py
```

## Troubleshooting

- **Isaac ROS packages not found**: Verify ROS 2 Humble is properly sourced and repositories are added
- **GPU not detected**: Check NVIDIA drivers and CUDA installation
- **Performance issues**: Verify Isaac ROS nodes are using GPU acceleration
- **Docker permission errors**: Add user to docker group with `sudo usermod -aG docker $USER`

## Next Steps

After successful installation of Isaac ROS packages:

1. Configure Isaac Sim-ROS 2 bridge with proper topic mappings
2. Set up sensor configurations for camera, LiDAR, and IMU in Isaac Sim
3. Test Isaac ROS perception nodes with sample data