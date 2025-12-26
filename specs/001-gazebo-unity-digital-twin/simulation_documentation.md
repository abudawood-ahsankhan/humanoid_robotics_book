# Gazebo & Unity Digital Twin Simulation System

## Overview

This document provides comprehensive documentation for the Gazebo & Unity Digital Twin simulation system for humanoid robots. The system integrates Gazebo physics simulation with Unity high-fidelity visualization, all connected through ROS 2 for complete control and monitoring capabilities.

## System Architecture

The simulation system consists of three main components:

1. **Gazebo Simulation Environment**: Provides accurate physics simulation including gravity, collisions, friction, and joint dynamics
2. **Unity Visualization**: Offers high-fidelity rendering and human-robot interaction visualization
3. **ROS 2 Integration**: Enables communication between all components and external control nodes

## Components

### 1. Robot Model (URDF)
- 8-DOF humanoid robot with arms, legs, and head
- Proper inertial properties for realistic physics simulation
- Gazebo plugins for sensors and control
- Transmission interfaces for ROS 2 control

### 2. Gazebo Worlds
- `empty.world`: Basic environment with floor and lighting
- `humanoid_room.world`: Room with floor and simple obstacles
- `obstacle_course.world`: Complex environment with walls and movable objects

### 3. Sensors
- **LiDAR**: 360-degree scanning with 10m range
- **Depth Camera**: 640x480 resolution with 60° FOV
- **IMU**: Inertial measurement unit with 100Hz update rate

### 4. ROS 2 Nodes
- `simulation_controller`: Synchronizes Gazebo and Unity
- `robot_state_publisher`: Publishes robot state transforms
- `joint_state_publisher`: Publishes joint positions

### 5. Unity Visualization
- Robot model with joint articulation
- ROS# communication interface
- Real-time synchronization with Gazebo

## Launch Files

### Basic Simulation
```bash
ros2 launch robot_simulation simulation.launch.py
```

### Complete Simulation
```bash
ros2 launch robot_simulation simulation_complete.launch.py
```

### Custom World
```bash
ros2 launch robot_simulation simulation.launch.py world:=path/to/world
```

## Topics and Services

### Published Topics
- `/joint_states` - Joint positions, velocities, efforts
- `/sensor_data/laser_scan` - LiDAR sensor data
- `/sensor_data/depth_camera/image` - Depth camera images
- `/sensor_data/imu` - IMU sensor data
- `/unity_visualization/robot_state` - Robot state for Unity

### Subscribed Topics
- `/joint_commands` - Joint position commands
- `/unity_visualization/commands` - Unity visualization commands

## Configuration

### Sensor Configuration
Located in `config/sensors.yaml`, this file allows adjustment of:
- Sensor ranges and resolutions
- Update rates
- Noise parameters
- Topic names

### World Configuration
World files in `worlds/` directory define:
- Environment geometry
- Physics properties
- Lighting conditions
- Obstacles and objects

## Usage Scenarios

### 1. Basic Simulation Setup
1. Launch Gazebo with the humanoid robot
2. Verify robot spawns correctly with physics properties
3. Check that joint states are published

### 2. Sensor Simulation
1. Start simulation with sensor plugins enabled
2. Verify sensor data publishing to ROS 2 topics
3. Monitor data using ROS tools like RViz

### 3. Unity Visualization
1. Launch Gazebo simulation
2. Start Unity scene with ROS# connection
3. Verify robot state synchronization between environments

### 4. Control Integration
1. Send joint commands from ROS 2 nodes
2. Observe robot response in Gazebo
3. Verify sensor feedback publishing
4. Confirm Unity visualization updates

## Troubleshooting

### Common Issues

1. **Robot not spawning in Gazebo**
   - Check URDF syntax and joint limits
   - Verify transmission interfaces
   - Ensure proper inertial properties

2. **Sensor data not publishing**
   - Verify Gazebo plugins are loaded
   - Check topic names and connections
   - Confirm sensor parameters in URDF

3. **Unity visualization not updating**
   - Check ROS# connection status
   - Verify topic subscriptions
   - Confirm message format compatibility

4. **Physics simulation instability**
   - Adjust physics parameters in world file
   - Verify joint limits and effort values
   - Check inertial properties in URDF

### Performance Tips

- Reduce update rates for sensors not in use
- Simplify collision geometry where high precision isn't needed
- Use appropriate world complexity for your hardware
- Monitor CPU and GPU usage during simulation

## Assessment Materials

### Exercise 1: Gazebo Simulation Setup
- Create a new world with custom obstacles
- Spawn the humanoid robot in the environment
- Verify physics properties (gravity response, collision detection)

### Exercise 2: Sensor Simulation
- Configure sensor parameters for different scenarios
- Monitor sensor data publication to ROS 2 topics
- Visualize sensor data in RViz

### Exercise 3: Unity Integration
- Connect Unity visualization to Gazebo simulation
- Verify real-time synchronization between environments
- Test joint control from ROS 2 nodes

### Exercise 4: Complete Integration
- Implement a complete control loop: ROS 2 → Gazebo → Sensors → ROS 2 → Unity
- Validate all system components work together
- Document any issues and solutions

## Success Criteria

1. Gazebo simulation environment runs smoothly with physics and sensors correctly simulated
2. Robot interacts naturally with the simulated environment with realistic physics behavior
3. Unity visualization accurately represents Gazebo simulation in real-time with synchronization delay under 100ms
4. ROS 2 nodes successfully control the simulated robot and receive sensor feedback with message latency under 50ms
5. All sensor simulation types (LiDAR, depth camera, IMU) produce realistic and accurate data matching expected sensor characteristics
6. Students can complete all hands-on exercises successfully with 90% success rate for basic robot movements and sensor data collection