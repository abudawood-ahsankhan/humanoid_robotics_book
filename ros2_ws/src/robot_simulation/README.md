# Robot Simulation

This package contains the simulation environment for the humanoid robot, including Gazebo worlds and launch files.

## World Files

### simple_room.world
A basic room environment for robot simulation with walls and lighting

## Launch Files

### simulation.launch.py
Launches Gazebo with the robot model and necessary controllers

## Usage

To launch the simulation:
```bash
# Launch Gazebo with the simple room world
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/simple_room.world

# Spawn the robot in Gazebo
ros2 run gazebo_ros spawn_entity.py -entity humanoid_robot -topic robot_description
```

## Configuration

The simulation package includes:
- Physics parameters
- World configurations
- Sensor simulation settings