# Isaac ROS Perception Pipeline Test Guide with Isaac Sim

## Overview

This guide provides instructions for testing the Isaac ROS perception pipeline with Isaac Sim sensor data. The test validates that the perception system is correctly processing data from Isaac Sim and producing expected outputs for object detection, semantic segmentation, and visual SLAM.

## Prerequisites

- Isaac Sim running with humanoid robot model
- Isaac ROS perception pipeline running
- Isaac Sim-ROS bridge configured and operational
- Robot with camera, LiDAR, and IMU sensors properly configured in Isaac Sim

## Test Setup

### 1. Environment Preparation

Ensure Isaac Sim is running with the humanoid navigation environment:

```bash
# Launch Isaac Sim with humanoid navigation scene
# In Isaac Sim, load the humanoid_navigation.usd world
```

### 2. System Preparation

Launch all required components:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch Isaac Sim bridge
ros2 launch isaac_simulation isaac_sim_bridge.launch.py

# Launch Isaac ROS perception pipeline
ros2 launch isaac_robot_perception perception_pipeline.launch.py
```

### 3. Verify Sensor Data

Confirm that Isaac Sim is publishing sensor data:

```bash
# Check available topics
ros2 topic list | grep -E "(sensors|perception|vslam)"

# Verify camera data
ros2 topic echo /sensors/camera/image_raw --field header.stamp -1

# Verify LiDAR data
ros2 topic echo /sensors/lidar/points --field header.stamp -1

# Verify IMU data
ros2 topic echo /sensors/imu/data --field header.stamp -1
```

## Test Execution

### 1. Running the Automated Test

```bash
# Run the perception pipeline test (default 60 seconds)
python3 src/isaac_robot_perception/test/test_perception_pipeline_with_isaac_sim.py
```

### 2. Custom Test Parameters

You can customize the test parameters using ROS parameters:

```bash
# Run with custom test duration and requirements
python3 src/isaac_robot_perception/test/test_perception_pipeline_with_isaac_sim.py --ros-args \
  -p test_duration:=120.0 \
  -p min_camera_rate:=15.0 \
  -p min_lidar_rate:=8.0 \
  -p max_latency:=0.05
```

### 3. Manual Testing

For manual testing of perception components:

1. **Object Detection**:
   - Verify detection topic is publishing: `ros2 topic echo /perception/objects`
   - Check that objects are detected in RViz with bounding boxes
   - Confirm detection accuracy with visual inspection

2. **Semantic Segmentation**:
   - Verify segmentation topic: `ros2 topic echo /perception/segmentation`
   - Check segmentation overlay in RViz
   - Confirm different object classes are properly segmented

3. **Visual SLAM**:
   - Monitor pose topic: `ros2 topic echo /vslam/pose`
   - Check map topic: `ros2 topic echo /vslam/map`
   - Verify localization in RViz with SLAM map

## Test Metrics

### 1. Data Rate Metrics
- **Camera Data Rate**: Should meet minimum requirement (≥10 Hz)
- **LiDAR Data Rate**: Should meet minimum requirement (≥5 Hz)
- **IMU Data Rate**: Should meet minimum requirement (≥50 Hz)
- **Detection Rate**: Should meet minimum requirement (≥5 Hz)
- **Segmentation Rate**: Should meet minimum requirement (≥5 Hz)

### 2. Latency Metrics
- **Camera Processing Latency**: Should be ≤0.1 seconds
- **LiDAR Processing Latency**: Should be ≤0.1 seconds
- **Detection Latency**: Should be ≤0.1 seconds
- **Segmentation Latency**: Should be ≤0.1 seconds
- **VSLAM Latency**: Should be ≤0.1 seconds

### 3. Accuracy Metrics
- **Detection Accuracy**: Quality of object detection
- **Segmentation Accuracy**: Quality of semantic segmentation
- **VSLAM Accuracy**: Localization and mapping quality

## Expected Results

### Successful Test Indicators
- All sensor data topics publishing regularly
- Detection rate ≥ 5 Hz with reasonable detection quality
- Segmentation rate ≥ 5 Hz with accurate class identification
- VSLAM pose updates ≥ 10 Hz with stable localization
- Latencies ≤ 0.1 seconds for all components
- No significant processing delays or dropped messages

### Performance Categories

#### Excellent Performance (>0.8)
- Data rates significantly exceeding minimum requirements
- Latencies < 0.05 seconds
- High accuracy in all perception tasks
- Stable performance throughout test duration

#### Good Performance (0.6-0.8)
- Data rates meeting minimum requirements
- Latencies < 0.1 seconds
- Good accuracy in perception tasks
- Mostly stable performance

#### Acceptable Performance (0.4-0.6)
- Data rates close to minimum requirements
- Latencies approaching maximum allowed
- Acceptable accuracy in perception tasks
- Some performance fluctuations acceptable

#### Poor Performance (<0.4)
- Data rates below minimum requirements
- High latencies exceeding maximum allowed
- Low accuracy in perception tasks
- Unstable performance

## Troubleshooting

### Common Issues

#### No Sensor Data
**Symptoms**: No sensor data being received from Isaac Sim

**Solutions**:
- Verify Isaac Sim-ROS bridge connection
- Check Isaac Sim world has sensors properly configured
- Confirm sensor topics are mapped correctly
- Verify Isaac Sim physics are enabled

#### Low Data Rates
**Symptoms**: Sensor data rates below requirements

**Solutions**:
- Check Isaac Sim rendering/physics settings
- Adjust sensor update frequencies in Isaac Sim
- Verify ROS bridge configuration
- Check system resource usage

#### High Latency
**Symptoms**: Processing latencies exceeding requirements

**Solutions**:
- Optimize perception pipeline parameters
- Check hardware acceleration (CUDA/TensorRT)
- Reduce sensor data resolution if needed
- Check for system bottlenecks

#### Poor Detection Quality
**Symptoms**: Low detection accuracy or missed objects

**Solutions**:
- Verify camera calibration parameters
- Check lighting conditions in simulation
- Adjust detection confidence thresholds
- Validate model weights and configuration

#### SLAM Instability
**Symptoms**: Unstable localization or mapping

**Solutions**:
- Check IMU integration in SLAM pipeline
- Verify camera calibration parameters
- Adjust SLAM parameters for stability
- Check for sensor synchronization issues

## Performance Optimization

### 1. Isaac Sim Optimization
- Adjust rendering quality settings for performance
- Optimize physics update rates
- Use appropriate sensor update frequencies
- Enable hardware acceleration features

### 2. Perception Pipeline Optimization
- Use Nitros for optimized data transfer
- Enable CUDA acceleration for neural networks
- Adjust neural network input sizes
- Use TensorRT optimization where possible

### 3. System Optimization
- Ensure adequate GPU resources
- Optimize memory usage with pinned memory
- Use appropriate QoS settings for sensor data
- Consider multi-threaded processing

## Verification Commands

Run these commands to verify perception pipeline operation:

```bash
# Check all perception topics are active
ros2 topic list | grep perception

# Monitor camera processing
ros2 run image_view image_view image:=/sensors/camera/image_raw

# Monitor detections
ros2 run rviz2 rviz2  # Add Detection2DArray display

# Monitor segmentation
ros2 run image_view image_view image:=/perception/segmentation

# Monitor VSLAM
ros2 run rviz2 rviz2  # Add Map and Pose displays

# Check system performance
nvidia-smi  # GPU usage
htop        # CPU usage
```

## Next Steps

After successful perception pipeline testing:

1. **Integrate with Navigation**: Connect perception outputs to navigation system
2. **Test with AI Training**: Use perception outputs for AI model training
3. **Validate in Complex Scenarios**: Test with more challenging environments
4. **Optimize Performance**: Fine-tune parameters for best performance
5. **Prepare for Real Robot**: Plan for transition to physical robot when available

## Integration with Navigation System

Once perception pipeline is validated:

1. Connect detection outputs to navigation obstacle avoidance
2. Use SLAM outputs for navigation localization
3. Integrate segmentation for terrain analysis
4. Test combined perception-navigation performance

## Safety Considerations

- Ensure perception pipeline doesn't overload system resources
- Monitor for any system instability during testing
- Have emergency stops available for robot control
- Verify sensor data integrity before using for navigation