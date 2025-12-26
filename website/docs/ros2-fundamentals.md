---
sidebar_position: 5
---

# ROS 2 Fundamentals

## Overview

ROS 2 (Robot Operating System 2) is the next-generation middleware for robotics applications. This section covers the essential concepts needed to understand and work with ROS 2 in the context of humanoid robotics.

## Architecture

ROS 2 uses a DDS (Data Distribution Service)-based communication architecture that provides:

- **Decentralized communication**: Nodes can communicate without a central master
- **Real-time capabilities**: Support for time-sensitive applications
- **Multi-language support**: Nodes can be written in different programming languages
- **Platform independence**: Runs on various operating systems

### Key Components

1. **Nodes**: Processes that perform computation. Nodes are the fundamental building blocks of a ROS 2 program.
2. **Topics**: Named buses over which nodes exchange messages. Topics use a publisher-subscriber communication model.
3. **Services**: Synchronous request-response communication pattern between nodes.
4. **Actions**: Asynchronous communication pattern for long-running tasks with feedback.

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. In our humanoid robotics implementation, we created several nodes:

- `joint_publisher_node`: Publishes joint state information
- `joint_subscriber_node`: Subscribes to joint commands
- `gesture_service_node`: Provides gesture execution services
- `movement_action_node`: Handles robot movement actions

### Node Lifecycle

Nodes in ROS 2 have a well-defined lifecycle:
- Unconfigured
- Inactive
- Active
- Finalized

## Topics and Publisher-Subscriber Model

Topics enable asynchronous communication through a publish-subscribe model. In our implementation:

- `joint_states` topic: Publishes current joint positions, velocities, and efforts
- `joint_command` topic: Receives commands to control joint positions

```python
# Example publisher from our implementation
publisher_ = self.create_publisher(JointState, 'joint_states', 10)

# Example subscriber from our implementation
subscription = self.create_subscription(
    JointCommand,
    'joint_command',
    self.joint_command_callback,
    10
)
```

## Services

Services provide synchronous request-response communication. Our gesture service allows triggering predefined gestures:

- `trigger_gesture` service: Accepts a gesture name and executes the corresponding movement

## Actions

Actions are designed for long-running tasks that provide feedback during execution. Our movement action enables:

- `move_robot` action: Moves the robot to a target pose with continuous feedback

## Quality of Service (QoS)

ROS 2 provides Quality of Service settings to control communication behavior, including reliability, durability, and history policies.

## Python Integration with rclpy

rclpy is the Python client library for ROS 2 that enables Python-based robot control:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Initialize publishers, subscribers, etc.
```

## Key Concepts for Humanoid Robotics

When working with humanoid robots in ROS 2, these concepts are particularly important:

- **Joint Control**: Managing multiple degrees of freedom
- **Sensor Integration**: Processing data from various sensors
- **Real-time Performance**: Ensuring timely responses
- **Safety Constraints**: Implementing safe robot behavior
- **Simulation Integration**: Working with simulation environments

## Best Practices

- Use appropriate QoS (Quality of Service) settings for different data types
- Implement proper error handling and recovery mechanisms
- Follow ROS 2 naming conventions
- Use launch files for complex system startup
- Separate concerns between different nodes