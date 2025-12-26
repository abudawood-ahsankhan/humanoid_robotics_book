# Quickstart Guide: ROS 2 Robotics Module

**Feature**: 001-ros2-robotics
**Date**: 2025-12-20

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Gazebo Garden
- Basic knowledge of Linux terminal

## Setup Instructions

### 1. Install ROS 2 Humble Hawksbill

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to apt sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-rclpy
sudo apt install ros-humble-ros2launch
```

### 2. Setup ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build workspace
colcon build
source install/setup.bash
```

### 3. Clone Robot Control Package

```bash
cd ~/ros2_ws/src
git clone [repository-url] robot_control_nodes
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the Robot Control Nodes

### 1. Launch Simulation Environment

```bash
# Launch robot simulation
ros2 launch robot_simulation robot_world.launch.py
```

### 2. Launch Robot Control Nodes

```bash
# Launch joint control node
ros2 run robot_control_nodes joint_control_node

# Launch gesture service node
ros2 run robot_control_nodes gesture_service_node

# Launch movement action node
ros2 run robot_control_nodes movement_action_node
```

### 3. Visualize in RViz

```bash
# Launch RViz with robot configuration
ros2 launch robot_description display.launch.py
```

## Basic Commands

### Publish Joint Commands

```bash
# Send joint position command
ros2 topic pub /joint_commands sensor_msgs/msg/JointState '{name: ["joint1"], position: [1.57]}'
```

### Call Robot Services

```bash
# Trigger predefined gesture
ros2 service call /trigger_gesture robot_control_interfaces/srv/GestureTrigger '{gesture_name: "wave"}'
```

### Send Action Goals

```bash
# Move robot to target pose
ros2 action send_goal /move_robot robot_control_interfaces/action/MoveRobot '{target_pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

## Troubleshooting

### Common Issues

1. **Package not found**: Ensure you've sourced the workspace (`source ~/ros2_ws/install/setup.bash`)

2. **Permission errors**: Check that ROS 2 is properly installed and environment is sourced

3. **Simulation not responding**: Verify Gazebo is running and robot model is loaded

4. **Nodes not communicating**: Check that all nodes are on the same ROS domain ID

### Useful Commands

```bash
# List active topics
ros2 topic list

# List active services
ros2 service list

# Check node status
ros2 node list

# Echo topic messages
ros2 topic echo /joint_states sensor_msgs/msg/JointState
```