# Data Model: Gazebo & Unity Digital Twin

## Robot Model Entity
- **Name**: Unique identifier for the robot instance
- **Description**: Human-readable description of the robot
- **URDF Path**: File path to the URDF model definition
- **Joint States**: Collection of joint names, positions, velocities, and efforts
- **Link States**: Collection of link poses (position and orientation)
- **Physical Properties**: Mass, inertia, friction, and collision properties
- **Relationships**: Connected to Simulation Environment, Sensor Data

## Simulation Environment Entity
- **Name**: Unique identifier for the environment
- **Description**: Human-readable description of the environment
- **World File**: Path to the Gazebo world file (SDF format)
- **Physics Properties**: Gravity, damping, and other physics parameters
- **Objects**: Collection of static and dynamic objects in the environment
- **Relationships**: Contains Robot Model, Sensor Data

## Sensor Data Entity
- **Type**: LiDAR, Depth Camera, IMU, or other sensor type
- **Topic Name**: ROS 2 topic where sensor data is published
- **Data Format**: Message type (e.g., sensor_msgs/LaserScan, sensor_msgs/Image, sensor_msgs/Imu)
- **Update Rate**: Frequency at which sensor data is published
- **Parameters**: Sensor-specific configuration parameters
- **Relationships**: Associated with Robot Model, Simulation Environment

## Joint Command Entity
- **Joint Names**: Array of joint names to control
- **Positions**: Target joint positions
- **Velocities**: Target joint velocities
- **Efforts**: Target joint efforts/torques
- **Timestamp**: Time when command was issued
- **Relationships**: Commands Robot Model

## Visualization State Entity
- **Robot Pose**: Position and orientation of the robot in Unity coordinate system
- **Joint Positions**: Current joint positions for animation in Unity
- **Update Frequency**: Rate at which visualization state is updated
- **Synchronization Status**: Whether Unity visualization matches Gazebo simulation
- **Relationships**: Mirrors Robot Model, Sensor Data

## Validation Rules
- Robot Model must have valid URDF format with all referenced mesh files existing
- Joint States must match the degrees of freedom defined in the URDF
- Simulation Environment must have valid SDF format
- Sensor Data must use standard ROS 2 message types
- Joint Command values must be within physical limits defined in URDF
- Visualization State must maintain synchronization within 100ms of simulation time

## State Transitions
- Robot Model: [Unloaded] → [Loaded] → [Simulated] → [Controlled] → [Visualized]
- Simulation Environment: [Empty] → [Configured] → [Running] → [Paused] → [Stopped]
- Sensor Data: [Inactive] → [Initialized] → [Publishing] → [Streaming] → [Stopped]