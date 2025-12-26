# Data Model: ROS 2 Robotics Module

**Feature**: 001-ros2-robotics
**Date**: 2025-12-20

## Key Entities

### JointState
**Description**: Represents the current state of robot joints including position, velocity, and effort
**Fields**:
- `name` (string[]): Names of the joints
- `position` (float64[]): Joint positions in radians
- `velocity` (float64[]): Joint velocities in radians/second
- `effort` (float64[]): Joint efforts in units of effort

**Validation**: Position values must be within joint limits defined in URDF
**State transitions**: Updated continuously during robot operation

### RobotDescription
**Description**: XML-based representation of robot structure and kinematics
**Fields**:
- `links` (array): Physical components of the robot
- `joints` (array): Connections between links with kinematic properties
- `materials` (array): Visual appearance properties
- `gazebo` (array): Simulation-specific properties

**Validation**: Must conform to URDF schema and maintain valid kinematic chain
**State transitions**: N/A - static configuration

### JointCommand
**Description**: Command to control a specific robot joint
**Fields**:
- `joint_name` (string): Name of the target joint
- `target_position` (float64): Desired position in radians
- `target_velocity` (float64): Desired velocity in radians/second
- `effort` (float64): Applied effort in units of effort

**Validation**: Command values must be within joint limits
**State transitions**: Transmitted → Executed → Applied

### RobotPose
**Description**: Represents the 3D position and orientation of the robot
**Fields**:
- `position` (object): x, y, z coordinates in meters
- `orientation` (object): quaternion (x, y, z, w) representation
- `timestamp` (time): Time of pose measurement

**Validation**: Orientation must be a normalized quaternion
**State transitions**: Updated during robot movement

### SensorData
**Description**: Data from robot sensors (IMU, cameras, force/torque)
**Fields**:
- `sensor_type` (string): Type of sensor (IMU, camera, force_torque, etc.)
- `sensor_name` (string): Unique name of the sensor
- `data` (object): Sensor-specific measurement data
- `timestamp` (time): Time of measurement

**Validation**: Data values must be within sensor range
**State transitions**: Acquired → Processed → Available

## Relationships

- `RobotDescription` contains multiple `JointState` elements
- `JointCommand` targets specific joints defined in `RobotDescription`
- `RobotPose` reflects the result of multiple `JointCommand` executions
- `SensorData` provides feedback about the state resulting from commands