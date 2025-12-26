# VSLAM Mapping Test Guide for Simple Office Environment

## Overview

This guide provides instructions for testing the Visual SLAM (VSLAM) mapping capabilities in the simple office environment. The test evaluates mapping accuracy, localization precision, and system stability during autonomous exploration.

## Prerequisites

- Isaac Sim running with simple office environment (`simple_office.usd`)
- Isaac ROS VSLAM pipeline running
- Robot properly configured with camera and localization nodes
- All necessary topics publishing data:
  - `/sensors/camera/image_raw` - Camera images
  - `/vslam/pose` - VSLAM estimated pose
  - `/vslam/map` - Generated occupancy map
  - `/vslam/path_optimized` - Optimized trajectory

## Test Setup

### 1. Environment Preparation

Ensure the simple office environment is loaded in Isaac Sim:

```bash
# Launch Isaac Sim with simple office environment
# Load the simple_office.usd world file
```

### 2. System Preparation

Launch all required components:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch VSLAM pipeline
ros2 launch isaac_robot_perception visual_slam_pipeline.launch.py

# Launch robot control for navigation (if testing autonomous exploration)
```

### 3. Robot Positioning

Position the robot at the starting location in the office environment, typically at a corner or center location that provides good visibility of the surroundings.

## Test Execution

### 1. Running the Automated Test

```bash
# Run the VSLAM mapping test (default 120 seconds)
python3 src/isaac_robot_perception/test/test_vslam_mapping.py
```

### 2. Custom Test Parameters

You can customize the test parameters using ROS parameters:

```bash
# Run with custom test duration (180 seconds) and stricter requirements
python3 src/isaac_robot_perception/test/test_vslam_mapping.py --ros-args \
  -p test_duration:=180.0 \
  -p min_map_coverage:=0.15 \
  -p max_position_error:=0.3 \
  -p min_features_tracked:=75
```

### 3. Manual Testing

For manual testing, you can also:

- Drive the robot manually through the office environment
- Monitor the generated map in RViz
- Observe pose estimation accuracy
- Check for loop closures and map consistency

## Test Metrics

### 1. Map Coverage
- **Definition**: Percentage of the environment that has been mapped
- **Target**: ≥ 10% of expected area (typically > 10% for simple office)
- **Measurement**: Ratio of occupied cells in occupancy grid

### 2. Feature Tracking Score
- **Definition**: Stability and quality of feature tracking
- **Target**: ≥ 50 tracked features per frame
- **Measurement**: Average number of features successfully tracked

### 3. Tracking Stability
- **Definition**: Consistency of pose estimation over time
- **Target**: Minimal drift and smooth trajectory
- **Measurement**: Average movement between consecutive poses

### 4. Mapping Completeness
- **Definition**: How thoroughly the environment has been explored
- **Target**: Coverage of most navigable areas
- **Measurement**: Path length and area coverage

## Expected Results

### Simple Office Environment Characteristics
- Dimensions: ~20m x 20m (400 m²)
- Contains: Walls, furniture (tables, chairs), doors
- Lighting: Consistent indoor lighting
- Textures: Varied textures for feature detection

### Success Criteria
- **Map Coverage**: ≥ 10% of environment mapped
- **Feature Tracking**: ≥ 50 features tracked per frame
- **Tracking Stability**: Smooth trajectory with minimal drift
- **Mapping Completeness**: ≥ 30% exploration of environment

## Troubleshooting

### Low Map Coverage

**Symptoms**: Small map, large unexplored areas

**Solutions**:
- Improve camera calibration
- Adjust feature detection parameters (ORB settings)
- Ensure adequate lighting in environment
- Verify camera mounting and field of view

### Poor Feature Tracking

**Symptoms**: Low feature count, frequent tracking failures

**Solutions**:
- Tune ORB detector parameters (nfeatures, scale_factor)
- Optimize image preprocessing
- Check camera focus and exposure
- Verify sufficient texture in environment

### High Position Drift

**Symptoms**: Inconsistent trajectory, poor localization

**Solutions**:
- Improve camera intrinsic/extrinsic calibration
- Enable sensor fusion with IMU
- Adjust VSLAM parameters (tracking thresholds)
- Check for loop closure functionality

### Mapping Inconsistencies

**Symptoms**: Ghost walls, misaligned map sections

**Solutions**:
- Verify loop closure detection is working
- Adjust pose graph optimization parameters
- Check for proper initialization
- Ensure sufficient overlap between views

## Performance Optimization Tips

### 1. Feature Detection
- Adjust ORB parameters for optimal feature count
- Balance between feature density and processing speed
- Consider environment-specific tuning

### 2. Mapping Resolution
- Set appropriate map resolution (typically 5-10cm)
- Balance between detail and memory usage
- Consider environment scale

### 3. Loop Closure
- Ensure loop closure detection is properly configured
- Adjust thresholds for optimal performance
- Verify pose graph optimization is active

## Visualization and Monitoring

### 1. RViz Setup
Add displays for:
- RobotModel (for robot visualization)
- OccupancyGrid (for the map)
- Path (for trajectory)
- Image (for camera feed)
- TF (for coordinate frames)

### 2. Key Topics to Monitor
- `/vslam/map` - Generated map
- `/vslam/pose` - Robot pose estimate
- `/vslam/path_optimized` - Optimized trajectory
- `/sensors/camera/image_raw` - Camera feed

## Next Steps

After successful VSLAM mapping test:

1. **Test in Complex Environments**: Move to more challenging scenarios
2. **Long-term Stability**: Test extended mapping sessions
3. **Multi-session Mapping**: Test map consistency across sessions
4. **Integration Testing**: Combine with navigation and planning
5. **Real-world Validation**: Test with physical robot if available

## Safety Considerations

- Ensure robot path is collision-free during autonomous testing
- Monitor robot position and stop if necessary
- Verify environment is safe for robot operation
- Have emergency stop procedures ready