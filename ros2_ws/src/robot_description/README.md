# Robot Description

This package contains the URDF description of the humanoid robot, including its kinematic structure, visual properties, and sensor definitions.

## URDF Files

### humanoid.urdf
Basic humanoid robot model with 8 DOF (4 in arms, 4 in legs)

### humanoid_complete.urdf
Enhanced humanoid robot model with additional sensors and gazebo plugins

## Structure

The robot consists of:
- Base link (mobile base)
- Torso
- Head
- Left and right arms (2 DOF each)
- Left and right legs (2 DOF each)

## Sensors

- Camera on the head
- IMU in the torso

## Launch Files

### display.launch.py
Launches the robot state publisher and visualizes the robot in RViz

## Usage

To visualize the robot in RViz:
```bash
ros2 launch robot_description display.launch.py
```

To view the robot with joint states:
```bash
# Terminal 1: Launch the robot description
ros2 launch robot_description display.launch.py

# Terminal 2: Publish some joint states (example)
ros2 topic pub /joint_states sensor_msgs/JointState "{name: ['left_shoulder_pitch'], position: [0.5]}"
```