# Robot Control Nodes

This package contains ROS 2 nodes for controlling the humanoid robot, including joint control, gesture services, and movement actions.

## Nodes

### Joint Publisher Node
- Publishes joint state messages for the robot
- Used for visualization and feedback

### Joint Subscriber Node
- Subscribes to joint command messages
- Updates robot joint positions based on commands

### Gesture Service Node
- Provides a service interface for predefined gestures
- Supports wave, nod, point, and dance gestures

### Movement Action Node
- Provides an action interface for robot movement
- Supports moving to target poses with feedback

### Gesture Client Node
- Client example for calling the gesture service

## Messages

### JointCommand
- `joint_name`: Name of the joint to control
- `target_position`: Desired joint position
- `target_velocity`: Desired joint velocity
- `effort`: Applied effort

## Services

### GestureTrigger
- `gesture_name`: Name of the gesture to execute
- `success`: Whether the gesture was executed successfully
- `message`: Additional information about the execution

## Actions

### MoveRobot
- Goal: Target pose to move to
- Feedback: Distance to goal and current status
- Result: Success status and completion message

## Launch Files

### joint_control.launch.py
Launches the joint publisher and subscriber nodes

### robot_services.launch.py
Launches the gesture service and movement action nodes

## Usage

To launch the joint control nodes:
```bash
ros2 launch robot_control_nodes joint_control.launch.py
```

To launch the robot services:
```bash
ros2 launch robot_control_nodes robot_services.launch.py
```