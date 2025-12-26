# Robot Spawning Test Guide

## Overview

This guide provides instructions for testing that the humanoid robot spawns correctly in Isaac Sim with proper rendering and physics properties.

## Prerequisites

- NVIDIA Isaac Sim installed and running
- Isaac Sim-ROS bridge configured and running
- Humanoid robot model imported into Isaac Sim
- Isaac ROS packages installed and configured

## Test Procedure

### 1. Pre-Test Setup

Before running the spawning test:

1. **Launch Isaac Sim**:
   - Start Isaac Sim application
   - Load or create a scene with the humanoid robot model
   - Ensure all required extensions are enabled

2. **Configure the Scene**:
   - Place the robot in the scene at the origin
   - Ensure proper lighting conditions
   - Verify physics properties are configured

3. **Launch ROS Bridge**:
   - Start the Isaac Sim-ROS bridge
   - Verify connection between Isaac Sim and ROS 2

### 2. Automated Test Execution

Run the automated test script:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
cd ~/humanoid_ws
source install/setup.bash

# Run the test script
python3 src/isaac_simulation/test/test_robot_spawning.py
```

### 3. Manual Verification Steps

In addition to automated tests, perform manual verification:

1. **Visual Inspection**:
   - Verify the robot appears in Isaac Sim
   - Check that all links are visible and properly connected
   - Confirm rendering looks photorealistic

2. **Physics Verification**:
   - Check that the robot responds to gravity appropriately
   - Verify that joints have proper limits and behavior
   - Ensure no unexpected collisions or interpenetrations

3. **Sensor Data Verification**:
   - Check that camera data is being published to `/sensors/camera/image_raw`
   - Verify LiDAR data is published to `/sensors/lidar/points`
   - Confirm IMU data is available on `/sensors/imu/data`
   - Verify joint states are published to `/isaac_sim/joint_states`

4. **TF Tree Verification**:
   - Use `ros2 run tf2_tools view_frames` to check the transform tree
   - Verify all robot links are properly connected
   - Confirm transforms are being published regularly

### 4. Command-Line Verification

Run these commands to verify robot spawning:

```bash
# Check available topics
ros2 topic list | grep -E "(sensors|joint|tf|odom)"

# Check topic types
ros2 topic info /sensors/camera/image_raw
ros2 topic info /isaac_sim/joint_states

# Monitor camera data
ros2 topic echo /sensors/camera/image_raw --field header.stamp

# Monitor joint states
ros2 topic echo /isaac_sim/joint_states --field position

# Check TF tree
ros2 run tf2_tools view_frames
```

### 5. RViz Visualization

Launch RViz to visualize the robot:

```bash
# Launch RViz
ros2 run rviz2 rviz2

# In RViz, add displays for:
# - RobotModel (set TF topic to /tf)
# - Image (set topic to /sensors/camera/image_raw)
# - TF (set topic to /tf)
# - LaserScan or PointCloud2 (set topic to /sensors/lidar/points)
```

## Expected Results

After successful robot spawning:

1. **Visual**: Robot appears in Isaac Sim with proper rendering
2. **Physics**: Robot responds to physics simulation appropriately
3. **Sensors**: All sensor topics are publishing data
4. **TF**: Transform tree shows complete robot kinematic chain
5. **Joint States**: Joint positions are being published regularly

## Troubleshooting

### Robot Not Visible in Isaac Sim
- Check that the USD model is properly loaded
- Verify the robot is not positioned outside the camera view
- Check material assignments and rendering settings

### No Sensor Data
- Verify Isaac Sim-ROS bridge is running
- Check topic mappings in bridge configuration
- Confirm sensor components are added to the robot in Isaac Sim

### Physics Issues
- Check that rigid body components are added to all links
- Verify joint limits and drive configurations
- Ensure proper mass and inertia properties

### TF Issues
- Verify the robot state publisher is running
- Check that joint state messages are being published
- Confirm proper frame IDs in sensor configurations

## Test Completion Criteria

The robot spawning test is considered successful when:

1. ✓ Robot model appears correctly in Isaac Sim
2. ✓ All sensor data topics are publishing regularly
3. ✓ TF tree shows complete robot kinematic chain
4. ✓ Physics simulation is stable and realistic
5. ✓ Rendering appears photorealistic

## Next Steps

After successful robot spawning verification:

1. Test perception pipeline with the spawned robot
2. Verify navigation capabilities in various environments
3. Test AI training with synthetic data from the robot
4. Validate sim-to-real transfer capabilities