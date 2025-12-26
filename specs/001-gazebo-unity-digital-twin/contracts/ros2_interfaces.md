# ROS 2 Interface Contracts: Gazebo & Unity Digital Twin

## Joint Control Interface

### Joint Trajectory Command
- **Topic**: `/joint_trajectory_controller/joint_trajectory`
- **Type**: `trajectory_msgs/JointTrajectory`
- **Direction**: Publisher (from control nodes to simulation)
- **Purpose**: Send joint position, velocity, and effort commands to the simulated robot

### Joint State Feedback
- **Topic**: `/joint_states`
- **Type**: `sensor_msgs/JointState`
- **Direction**: Subscriber (from simulation to visualization/control)
- **Purpose**: Provide current joint positions, velocities, and efforts from the simulation

## Sensor Data Interfaces

### LiDAR Sensor
- **Topic**: `/sensor_data/laser_scan`
- **Type**: `sensor_msgs/LaserScan`
- **Direction**: Publisher (from simulation to visualization/processing)
- **Purpose**: Provide simulated LiDAR data for environment perception

### Depth Camera
- **Topic**: `/sensor_data/depth_camera/image`
- **Type**: `sensor_msgs/Image`
- **Direction**: Publisher (from simulation to processing)
- **Purpose**: Provide simulated depth camera data for 3D perception

### IMU Sensor
- **Topic**: `/sensor_data/imu`
- **Type**: `sensor_msgs/Imu`
- **Direction**: Publisher (from simulation to processing)
- **Purpose**: Provide simulated inertial measurement data

## Simulation Control Interface

### Simulation Control Commands
- **Topic**: `/simulation_control`
- **Type**: `std_msgs/String`
- **Direction**: Publisher (from control nodes to simulation)
- **Purpose**: Send commands to control simulation state (start, stop, reset, pause)

### Simulation State Feedback
- **Topic**: `/simulation_state`
- **Type**: `std_msgs/String`
- **Direction**: Publisher (from simulation to monitoring)
- **Purpose**: Provide current simulation state information

## Visualization Interface

### Robot State for Unity
- **Topic**: `/unity_visualization/robot_state`
- **Type**: `sensor_msgs/JointState`
- **Direction**: Publisher (from simulation to Unity)
- **Purpose**: Provide robot joint states for Unity visualization synchronization

### Visualization Commands
- **Topic**: `/unity_visualization/commands`
- **Type**: `std_msgs/String`
- **Direction**: Publisher (from Unity to simulation)
- **Purpose**: Send visualization-specific commands (e.g., camera angles, lighting)

## Service Interfaces

### Robot Reset Service
- **Service**: `/reset_robot`
- **Type**: `std_srvs/Empty`
- **Purpose**: Reset robot to initial position in simulation

### Environment Change Service
- **Service**: `/change_environment`
- **Type**: `std_srvs/Trigger`
- **Purpose**: Change the simulation environment/world

### Sensor Configuration Service
- **Service**: `/configure_sensors`
- **Type**: `std_srvs/SetBool`
- **Purpose**: Enable/disable sensor simulation