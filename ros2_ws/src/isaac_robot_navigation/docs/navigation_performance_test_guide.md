# Navigation Performance Test Guide for Humanoid Robot

## Overview

This guide provides instructions for testing the navigation performance of the humanoid robot in the humanoid_navigation environment. The test evaluates path planning, obstacle avoidance, and bipedal locomotion capabilities under realistic conditions.

## Prerequisites

- Isaac Sim running with humanoid_navigation environment
- Isaac ROS navigation stack running
- Robot properly configured with navigation nodes
- All necessary topics publishing data:
  - `/odom` - Odometry data
  - `/cmd_vel` - Velocity commands
  - `/plan` - Planned path
  - `/scan` - Laser scan data
  - `/navigate_to_pose` - Navigation goal topic

## Humanoid Navigation Environment Characteristics

### Environmental Features
- **Dimensions**: ~10m x 10m (100 m²)
- **Layout**: Mixed indoor environment with corridors, rooms, and obstacles
- **Challenges**: Doorways, narrow passages, static and dynamic obstacles
- **Terrain**: Varied floor types suitable for bipedal locomotion
- **Obstacles**: Furniture, walls, simulated dynamic obstacles

### Navigation Challenges
- **Narrow passages**: Doorways and corridors requiring precise navigation
- **Turning constraints**: Humanoid robot turning radius limitations
- **Balance considerations**: Maintaining stability during navigation
- **Step constraints**: Limited step size and height for bipedal locomotion
- **Obstacle avoidance**: Dynamic and static obstacle navigation

## Test Setup

### 1. Environment Preparation

Ensure the humanoid navigation environment is loaded in Isaac Sim:

```bash
# Launch Isaac Sim with humanoid navigation environment
# Load the humanoid_navigation.usd world file
```

### 2. System Preparation

Launch all required components:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch navigation stack
ros2 launch isaac_robot_navigation navigation.launch.py

# Launch robot control for navigation
```

### 3. Robot Positioning

Position the robot at the starting location in the navigation environment, typically at a central hub area with good connectivity to different paths.

## Test Execution

### 1. Running the Automated Test

```bash
# Run the navigation performance test (default 300 seconds)
python3 src/isaac_robot_navigation/test/navigation_performance_test.py
```

### 2. Custom Test Parameters

You can customize the test parameters using ROS parameters:

```bash
# Run with custom test duration and requirements
python3 src/isaac_robot_navigation/test/navigation_performance_test.py --ros-args \
  -p test_duration:=600.0 \
  -p min_success_rate:=0.9 \
  -p max_execution_time:=120.0 \
  -p max_collision_count:=0
```

### 3. Manual Testing

For manual testing in the navigation environment:

- Navigate through different types of passages (wide, narrow, doorways)
- Test obstacle avoidance in various scenarios
- Evaluate bipedal locomotion stability
- Test path planning in complex layouts
- Assess recovery behaviors from navigation failures

## Test Metrics

### 1. Success Rate
- **Definition**: Percentage of navigation tasks completed successfully
- **Target**: ≥ 80% success rate
- **Measurement**: (successful_navigations / total_tests) * 100

### 2. Navigation Speed
- **Definition**: Average speed of navigation tasks
- **Target**: ≥ 0.1 m/s (minimum for practical navigation)
- **Measurement**: Average of linear velocity over navigation tasks

### 3. Collision Rate
- **Definition**: Frequency of collisions during navigation
- **Target**: ≤ 1 collision per test session
- **Measurement**: Number of collision events detected

### 4. Bipedal Stability
- **Definition**: Stability of robot during navigation
- **Target**: ≥ 0.7 stability score
- **Measurement**: Smoothness of motion and balance maintenance

### 5. Path Efficiency
- **Definition**: Efficiency of path planning and execution
- **Target**: High path efficiency score
- **Measurement**: Ratio of optimal path to actual path length

## Expected Results in Humanoid Navigation Environment

### Performance Categories

#### Excellent Performance (>0.8)
- High success rate (>90%)
- Good navigation speed (>0.2 m/s)
- No collisions
- Excellent bipedal stability (>0.8)
- Efficient path planning

#### Good Performance (0.6-0.8)
- Good success rate (80-90%)
- Adequate navigation speed (0.15-0.25 m/s)
- Minimal collisions (≤1)
- Good bipedal stability (0.7-0.8)
- Good path efficiency

#### Acceptable Performance (0.4-0.6)
- Acceptable success rate (70-80%)
- Basic navigation speed (0.1-0.15 m/s)
- Few collisions (≤2)
- Moderate stability (0.6-0.7)
- Moderate path efficiency

#### Poor Performance (<0.4)
- Low success rate (<70%)
- Slow navigation speed (<0.1 m/s)
- Frequent collisions (>2)
- Poor stability (<0.6)
- Inefficient path planning

## Troubleshooting Navigation Issues

### Low Success Rate

**Symptoms**: Frequent navigation failures

**Solutions**:
- Improve path planner parameters
- Adjust costmap inflation settings
- Optimize local planner configuration
- Enhance obstacle detection sensitivity
- Improve map quality and resolution

### Slow Navigation Speed

**Symptoms**: Robot moves very slowly

**Solutions**:
- Increase velocity limits in local planner
- Optimize trajectory generation parameters
- Improve obstacle detection efficiency
- Adjust acceleration/deceleration limits
- Consider terrain-specific speed profiles

### Frequent Collisions

**Symptoms**: Robot collides with obstacles frequently

**Solutions**:
- Increase safety margins in costmaps
- Improve obstacle detection range and resolution
- Optimize local planner obstacle avoidance
- Enhance sensor fusion for better obstacle detection
- Adjust robot footprint for accurate collision checking

### Poor Bipedal Stability

**Symptoms**: Unstable movement, wobbling, falls

**Solutions**:
- Optimize bipedal locomotion controller
- Improve balance feedback systems
- Adjust step timing and size for stability
- Implement gait adaptation for different terrains
- Add IMU feedback for balance maintenance

## Performance Optimization for Humanoid Navigation

### 1. Path Planning Optimization
- Use kinematically constrained path planners
- Implement step-constrained planning for bipedal locomotion
- Optimize for humanoid-specific turning radius
- Consider energy efficiency in path planning

### 2. Obstacle Avoidance
- Implement predictive obstacle avoidance
- Use multiple sensors for robust detection
- Consider dynamic obstacles in planning
- Implement safe stopping distances

### 3. Bipedal Locomotion
- Optimize step timing and size
- Implement adaptive gait control
- Use balance feedback for stability
- Consider terrain characteristics

### 4. Computational Efficiency
- Optimize path planning algorithms
- Use efficient obstacle detection methods
- Implement multi-threading for parallel processing
- Optimize sensor data processing

## Visualization and Monitoring

### 1. RViz Setup for Navigation
Add displays for:
- RobotModel (for robot visualization)
- Path (for planned and executed paths)
- OccupancyGrid (for costmap visualization)
- LaserScan (for obstacle detection)
- MarkerArray (for navigation goals and feedback)
- Odometry (for robot trajectory)

### 2. Key Topics to Monitor
- `/move_base/global_plan` - Global path plan
- `/move_base/local_plan` - Local path plan
- `/move_base/current_goal` - Current navigation goal
- `/odom` - Robot odometry
- `/cmd_vel` - Velocity commands
- `/scan` - Laser scan data
- `/tf` - Transform frames

### 3. Special Monitoring for Humanoid Navigation
- Footstep planning (if implemented)
- Balance metrics and stability indicators
- Joint position tracking during locomotion
- Energy consumption during navigation
- Terrain adaptability metrics

## Safety Considerations

- Ensure robot path avoids collisions with obstacles
- Monitor robot stability and stop if unstable
- Verify environment is safe for robot operation
- Have emergency stop procedures ready
- Plan safe navigation routes avoiding hazardous areas
- Consider fall recovery procedures

## Comparison with Other Environments

### Expected Performance Differences
- **Success Rate**: May be lower than simple environments due to complexity
- **Speed**: Slower due to bipedal constraints and obstacle avoidance
- **Stability**: More critical than for wheeled robots
- **Path Efficiency**: May be lower due to kinematic constraints

### Adaptation Strategies
- Adjust success criteria appropriately
- Focus on stability over speed
- Emphasize obstacle avoidance
- Implement humanoid-specific optimizations

## Next Steps After Testing

### 1. Performance Analysis
- Compare results with other environments
- Identify specific navigation challenges
- Document performance differences
- Create performance profiles for different scenarios

### 2. System Improvements
- Implement navigation-specific optimizations
- Enhance obstacle avoidance capabilities
- Optimize bipedal locomotion control
- Improve path planning algorithms

### 3. Integration Testing
- Test with perception system in navigation environment
- Validate localization during navigation
- Test long-term navigation stability
- Evaluate multi-session consistency

### 4. Real-world Validation
- Plan for testing with physical humanoid robot
- Consider sim-to-real transfer challenges
- Validate sensor configurations for real environments
- Assess practical deployment feasibility