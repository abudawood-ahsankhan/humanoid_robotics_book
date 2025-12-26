# Isaac ROS Perception Pipeline Performance Testing Guide

## Overview

This guide provides instructions for testing the real-time processing performance of the Isaac ROS perception pipeline with hardware acceleration. The performance test evaluates whether the perception pipeline meets real-time requirements for camera, LiDAR, and IMU processing with acceptable latency.

## Prerequisites

- Isaac Sim running with humanoid robot model
- Isaac ROS perception pipeline running
- All sensor topics publishing data (`/sensors/camera/image_raw`, `/sensors/lidar/points`, `/sensors/imu/data`)
- Object detection pipeline running (`/perception/objects`)

## Test Components

The performance test evaluates:

1. **Processing Frequency**: Whether the pipeline processes data at the required frequency
2. **Latency**: The time delay between sensor data generation and processed output
3. **System Resource Usage**: CPU, GPU, and memory utilization
4. **Hardware Acceleration**: Effectiveness of GPU acceleration

## Running the Performance Test

### 1. Prepare the System

Ensure all perception nodes are running:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch the perception pipeline
ros2 launch isaac_robot_perception perception_pipeline.launch.py
```

### 2. Run the Performance Test

```bash
# Run the performance test (default 30 seconds)
python3 src/isaac_robot_perception/test/performance_test.py
```

### 3. Custom Test Parameters

You can customize the test parameters using ROS parameters:

```bash
# Run with custom test duration (60 seconds)
python3 src/isaac_robot_perception/test/performance_test.py --ros-args -p test_duration:=60.0

# Run with custom frequency expectations
python3 src/isaac_robot_perception/test/performance_test.py --ros-args \
  -p expected_camera_frequency:=60.0 \
  -p expected_lidar_frequency:=20.0 \
  -p expected_imu_frequency:=200.0 \
  -p latency_threshold:=0.02
```

## Performance Metrics

### Frequency Requirements

The perception pipeline should achieve these minimum frequencies:

- **Camera Processing**: 30 Hz (real-time video processing)
- **LiDAR Processing**: 10 Hz (sufficient for navigation)
- **IMU Processing**: 100 Hz (for accurate state estimation)
- **Object Detection**: 15 Hz (real-time object recognition)

### Latency Requirements

Maximum acceptable latencies:

- **Camera Processing**: 50 ms (3 frames at 30 Hz)
- **LiDAR Processing**: 100 ms (1 frame at 10 Hz)
- **IMU Processing**: 20 ms (5 frames at 100 Hz)
- **Object Detection**: 70 ms (for real-time response)

### Resource Usage Targets

- **CPU Usage**: < 80% average
- **GPU Usage**: > 10% (indicating hardware acceleration is active)
- **Memory Usage**: < 80% of available memory

## Interpreting Results

The performance test report includes:

1. **Sensor-specific metrics**:
   - Average processing frequency
   - Frequency range (min/max)
   - Whether frequency requirements were met
   - Average latency
   - Latency range
   - Whether latency requirements were met

2. **System performance**:
   - CPU usage statistics
   - GPU usage statistics
   - Memory usage statistics

3. **Overall assessment**:
   - Whether real-time performance was achieved
   - Whether latency requirements were met
   - Whether hardware acceleration was effective

## Troubleshooting Performance Issues

### Low Processing Frequency

**Symptoms**: Processing frequency below requirements

**Solutions**:
- Check system resources (CPU, GPU, memory)
- Reduce input data resolution (e.g., camera resolution)
- Optimize processing pipeline
- Check for bottlenecks in specific nodes

### High Latency

**Symptoms**: Latency above threshold

**Solutions**:
- Optimize data transfer between nodes
- Reduce processing complexity
- Use Nitros for optimized data transport
- Check network/IPC performance

### Inadequate Hardware Acceleration

**Symptoms**: GPU usage < 10%

**Solutions**:
- Verify CUDA installation and GPU availability
- Check Isaac ROS GPU acceleration settings
- Ensure models are configured for GPU execution
- Verify Isaac ROS components support GPU

## Performance Optimization Tips

### 1. Pipeline Optimization
- Use Nitros for optimized data transport between nodes
- Minimize data copying between nodes
- Use appropriate QoS settings for each sensor type

### 2. Hardware Acceleration
- Ensure Isaac ROS components are configured for GPU
- Use appropriate CUDA device IDs
- Optimize batch sizes for GPU processing

### 3. Resource Management
- Monitor and limit resource usage
- Use appropriate thread priorities
- Optimize memory allocation patterns

## Expected Results

A well-optimized Isaac ROS perception pipeline should achieve:

- ✓ Camera processing at 30+ Hz with < 50ms latency
- ✓ LiDAR processing at 10+ Hz with < 100ms latency
- ✓ IMU processing at 100+ Hz with < 20ms latency
- ✓ Object detection at 15+ Hz with < 70ms latency
- ✓ GPU usage > 10% indicating hardware acceleration
- ✓ CPU usage < 80% for system stability

## Next Steps

After successful performance testing:

1. **Validate Perception Quality**: Ensure processing quality is maintained at high speeds
2. **Test Under Load**: Test with more complex scenes and objects
3. **Profile Individual Components**: Identify bottlenecks in specific nodes
4. **Optimize Further**: Apply additional optimizations based on profiling results
5. **Document Results**: Record performance characteristics for system design