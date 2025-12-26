# Localization Accuracy Test Guide for VSLAM System

## Overview

This guide provides instructions for testing the localization accuracy of the VSLAM system to ensure it meets the <10cm accuracy requirement. The test compares VSLAM estimated poses with ground truth poses to calculate localization errors.

## Prerequisites

- Isaac Sim running with ground truth pose publishing
- Isaac ROS VSLAM pipeline running
- Robot properly configured with camera and localization nodes
- Ground truth pose topic available: `/ground_truth/pose`
- VSLAM estimated pose topic: `/vslam/pose`
- Both topics publishing synchronized data

## Test Setup

### 1. Ground Truth Configuration

Ensure Isaac Sim is configured to publish ground truth poses:

```bash
# In Isaac Sim, ensure ground truth pose publisher is active
# The ground truth topic should publish PoseStamped messages
```

### 2. System Preparation

Launch all required components:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch VSLAM pipeline
ros2 launch isaac_robot_perception visual_slam_pipeline.launch.py

# Verify both topics are publishing
ros2 topic echo /ground_truth/pose --field pose.position -1
ros2 topic echo /vslam/pose --field pose.position -1
```

### 3. Robot Positioning

Position the robot at known locations in the environment for initial validation. Ensure the robot has a clear view of distinctive features for good VSLAM tracking.

## Test Execution

### 1. Running the Automated Test

```bash
# Run the localization accuracy test (default 120 seconds)
python3 src/isaac_robot_perception/test/localization_accuracy_test.py
```

### 2. Custom Test Parameters

You can customize the test parameters using ROS parameters:

```bash
# Run with custom test duration and stricter requirements
python3 src/isaac_robot_perception/test/localization_accuracy_test.py --ros-args \
  -p test_duration:=180.0 \
  -p error_threshold:=0.05 \
  -p success_threshold:=0.95
```

### 3. Manual Testing

For manual testing, you can also:

- Drive the robot through known paths
- Monitor the difference between ground truth and estimated poses
- Use RViz to visualize both poses simultaneously
- Record specific locations where accuracy is poor

## Test Metrics

### 1. Mean Position Error
- **Definition**: Average Euclidean distance between estimated and ground truth positions
- **Target**: < 0.1m (10cm) for overall requirement
- **Measurement**: Average of all position error samples

### 2. Max Position Error
- **Definition**: Maximum single error observed during test
- **Target**: < 0.1m (10cm) for overall requirement
- **Measurement**: Maximum of all position error samples

### 3. Success Rate
- **Definition**: Percentage of samples with error ≤ threshold
- **Target**: ≥ 90% of samples under 10cm threshold
- **Measurement**: (samples under threshold) / (total samples)

### 4. Error Distribution
- **Definition**: Statistical distribution of localization errors
- **Target**: Consistent performance with low variance
- **Measurement**: Standard deviation of position errors

## Accuracy Requirements

### Primary Requirements
- **Mean Error**: < 0.1m (10cm)
- **Max Error**: < 0.1m (10cm)
- **Success Rate**: ≥ 90% of samples under 10cm
- **Standard Deviation**: < 0.05m for consistency

### Secondary Requirements
- **Median Error**: < 0.08m (8cm)
- **95th Percentile**: < 0.15m (15cm)
- **Repeatability**: Consistent performance across multiple runs

## Expected Results

### Good Performance Indicators
- Mean error: < 5cm
- Success rate: > 95%
- Max error: < 8cm
- Low standard deviation: < 2cm

### Acceptable Performance Indicators
- Mean error: 5-10cm
- Success rate: 90-95%
- Max error: 8-15cm
- Standard deviation: 2-5cm

### Poor Performance Indicators
- Mean error: > 10cm
- Success rate: < 90%
- Max error: > 15cm
- High standard deviation: > 5cm

## Troubleshooting

### High Mean Error

**Symptoms**: Consistently poor localization accuracy

**Solutions**:
- Verify camera intrinsic calibration
- Check camera extrinsic calibration (mounting position)
- Improve feature detection parameters
- Ensure good lighting conditions
- Verify sufficient texture in environment

### High Max Error

**Symptoms**: Occasional large errors in localization

**Solutions**:
- Improve loop closure detection
- Enhance pose graph optimization
- Add sensor fusion (IMU, wheel encoders)
- Implement outlier rejection in tracking
- Check for motion blur in camera images

### Low Success Rate

**Symptoms**: Many samples exceed the 10cm threshold

**Solutions**:
- Retune VSLAM parameters
- Improve feature matching thresholds
- Enhance map quality
- Consider environment-specific optimization
- Add additional sensors for robustness

### High Variance

**Symptoms**: Inconsistent localization performance

**Solutions**:
- Stabilize camera mounting
- Improve image preprocessing
- Optimize feature tracking parameters
- Enhance pose graph optimization
- Check for sensor synchronization issues

## Performance Optimization Tips

### 1. Camera Calibration
- Perform accurate intrinsic calibration
- Verify extrinsic calibration (position/orientation on robot)
- Check for lens distortion correction
- Ensure consistent focus and exposure

### 2. Feature Detection
- Optimize ORB/SIFT parameters for your environment
- Adjust feature density for optimal tracking
- Implement dynamic parameter adjustment
- Consider environment-specific feature settings

### 3. Loop Closure
- Ensure robust loop closure detection
- Optimize geometric verification parameters
- Verify pose graph optimization is active
- Consider vocabulary-based relocalization

### 4. Sensor Fusion
- Integrate IMU data for motion prediction
- Consider wheel encoder integration
- Implement multi-sensor fusion
- Use Kalman filtering for state estimation

## Visualization and Monitoring

### 1. RViz Setup
Add displays for:
- Pose (for ground truth pose)
- Pose (for VSLAM estimated pose)
- Path (for trajectory comparison)
- Marker (for error visualization)

### 2. Key Topics to Monitor
- `/ground_truth/pose` - Ground truth position
- `/vslam/pose` - Estimated position
- `/tf` - Transform between frames
- `/vslam/map` - Generated map for context

### 3. Error Visualization
```bash
# Monitor error in real-time
ros2 run tf2_tools view_frames
ros2 run rqt_plot rqt_plot /localization_error
```

## Next Steps

After successful localization accuracy test:

1. **Test in Different Environments**: Validate in various scenarios
2. **Long-term Stability**: Test extended localization sessions
3. **Multi-session Consistency**: Test map reuse across sessions
4. **Integration Testing**: Combine with navigation and planning
5. **Real-world Validation**: Test with physical robot if available

## Safety Considerations

- Ensure robot operation is safe during testing
- Monitor robot position and stop if necessary
- Verify environment is safe for robot operation
- Have emergency stop procedures ready
- Avoid testing in areas with poor localization performance

## Quality Assurance

### 1. Test Reproducibility
- Run multiple test sessions
- Compare results across sessions
- Document any variations
- Ensure consistent conditions

### 2. Baseline Comparison
- Compare with previous implementations
- Document performance improvements
- Track regression if any
- Maintain performance metrics

### 3. Edge Case Testing
- Test at environment boundaries
- Test in low-texture areas
- Test with dynamic objects
- Test under varying lighting conditions