---
sidebar_position: 8
---

# Development Workflow

## Overview

This section outlines the recommended development workflow for working with humanoid robotics projects using ROS 2. The workflow encompasses the entire development lifecycle from initial concept to testing and deployment in simulation.

## Recommended Workflow

The development process follows these key stages:

1. **Specification**: Define requirements and architecture
2. **Design**: Create detailed implementation plans
3. **Implementation**: Develop the actual code
4. **Testing**: Validate functionality
5. **Iteration**: Refine based on testing results

## Setting Up Your Development Environment

### Prerequisites

- Ubuntu 22.04 LTS (recommended for ROS 2 Humble)
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Git for version control
- Basic understanding of Linux command line

### Workspace Setup

```bash
# Create a ROS 2 workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Creating a New Package

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python my_robot_package
```

## Development Cycle

### 1. Planning and Specification

Before writing code, clearly define:

- What the node/service/action should do
- Input and output requirements
- Expected behavior and edge cases
- Integration points with other components

### 2. Implementation

#### Creating Nodes

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        # Initialize components

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

#### Testing Individual Components

Test each component in isolation before integration:

```bash
# Build your package
cd ~/humanoid_ws
colcon build --packages-select my_robot_package

# Source the workspace
source install/setup.bash

# Run your node
ros2 run my_robot_package my_robot_node
```

### 3. Integration and Testing

#### Launch Files

Create launch files to start multiple nodes simultaneously:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_robot_node',
            name='my_robot_node',
            output='screen',
        ),
        Node(
            package='another_package',
            executable='another_node',
            name='another_node',
            output='screen',
        ),
    ])
```

#### Running Launch Files

```bash
ros2 launch my_robot_package my_launch_file.py
```

## Debugging Strategies

### Using ROS 2 Tools

- `ros2 topic list`: View available topics
- `ros2 topic echo /topic_name`: Monitor topic messages
- `ros2 service list`: View available services
- `ros2 node list`: View active nodes
- `rqt_graph`: Visualize node connections

### Logging

Use appropriate logging levels:

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        self.get_logger().info('Node initialized')
        self.get_logger().warn('Warning message')
        self.get_logger().error('Error message')
```

## Simulation Workflow

### Starting Gazebo Simulation

```bash
# Launch robot simulation
ros2 launch my_robot_description gazebo.launch.py

# Launch robot control nodes
ros2 launch my_robot_control control.launch.py
```

### Monitoring Simulation

- Use RViz for visualization
- Monitor joint states with `ros2 topic echo /joint_states`
- Check TF transforms with `tf2_tools view_frames`

## Version Control Best Practices

### Git Workflow

```bash
# Create a feature branch
git checkout -b feature/my_new_feature

# Make changes and commit
git add .
git commit -m "Add new robot control functionality"

# Push changes
git push origin feature/my_new_feature
```

### Commit Messages

Follow conventional commit format:

- `feat`: New features
- `fix`: Bug fixes
- `docs`: Documentation changes
- `test`: Adding or modifying tests
- `refactor`: Code restructuring without changing functionality

## Testing Methodology

### Unit Testing

Test individual functions and methods:

```python
import unittest
from my_robot_package.my_module import my_function

class TestMyFunction(unittest.TestCase):
    def test_my_function(self):
        result = my_function(input_value)
        self.assertEqual(result, expected_value)
```

### Integration Testing

Test the interaction between multiple components:

```bash
# Test that nodes can communicate properly
ros2 topic echo /expected_topic &
ros2 run package node_that_publishes &
# Verify messages are received as expected
```

## Continuous Integration

### Local Testing

Always run tests before pushing:

```bash
# Build and test
colcon build
colcon test
colcon test-result --verbose
```

## Performance Optimization

### Profiling

Identify bottlenecks in your code:

- Monitor CPU usage with `htop`
- Check network traffic for topic communication
- Use ROS 2 tools to monitor message rates

### Resource Management

- Use appropriate QoS settings for different topics
- Implement proper cleanup in node destruction
- Consider computational requirements for real-time performance

## Safety Considerations

### Simulation-First Approach

- Test all behaviors in simulation before considering real hardware
- Implement safety constraints in simulation environment
- Gradually increase complexity of tests

### Error Handling

Implement robust error handling:

```python
try:
    # Robot control code
    self.move_robot_to_position(target_pose)
except RobotControlException as e:
    self.get_logger().error(f'Robot control failed: {e}')
    # Implement safe recovery behavior
    self.emergency_stop()
```

## Best Practices Summary

1. **Plan before implementing**: Clearly define requirements and behavior
2. **Test early and often**: Validate components individually
3. **Use appropriate tools**: Leverage ROS 2 debugging and monitoring tools
4. **Document your work**: Include comments and documentation
5. **Follow ROS 2 conventions**: Naming, structure, and best practices
6. **Version control**: Use Git with meaningful commit messages
7. **Safety first**: Always test in simulation before real hardware
8. **Iterate and improve**: Refine based on testing results

## In Our Implementation

In our humanoid robotics implementation, we followed this workflow:

- Created separate packages for control, perception, and simulation
- Used launch files to coordinate multiple nodes
- Implemented comprehensive error handling
- Designed modular architecture for easy testing
- Followed ROS 2 best practices throughout

## Package Structure

A typical ROS 2 package follows this structure:

```
robot_control_nodes/          # Package name
├── CMakeLists.txt           # Build configuration (for C++)
├── package.xml              # Package metadata and dependencies
├── setup.py                 # Python package configuration
├── setup.cfg                # Installation configuration
├── resource/                # Resource files
├── launch/                  # Launch files
│   ├── joint_control.launch.py
│   └── robot_services.launch.py
├── nodes/                   # Python node implementations
│   ├── joint_publisher_node.py
│   ├── joint_subscriber_node.py
│   ├── gesture_service_node.py
│   └── movement_action_node.py
├── msg/                     # Custom message definitions
│   └── JointCommand.msg
├── srv/                     # Custom service definitions
│   └── GestureTrigger.srv
├── action/                  # Custom action definitions
│   └── MoveRobot.action
├── config/                  # Configuration files
└── test/                    # Test files
```

## Package.xml Configuration

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_control_nodes</name>
  <version>0.0.0</version>
  <description>ROS 2 nodes for controlling humanoid robot joints and movements</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>message_generation</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Custom Message, Service, and Action Definitions

### Messages (.msg)

Messages define the data structure for topics:

```
# JointCommand.msg
string joint_name
float64 target_position
float64 target_velocity
float64 effort
```

### Services (.srv)

Services define request-response data structures:

```
# GestureTrigger.srv
string gesture_name
---
bool success
string message
```

### Actions (.action)

Actions define goal-feedback-result structures:

```
# MoveRobot.action
geometry_msgs/Pose target_pose
---
bool success
string message
---
float64 distance_to_goal
string current_status
```

## Building and Running

### Building the Workspace

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/ros2_ws

# Build packages
colcon build

# Source the workspace
source install/setup.bash
```

## Parameter Configuration

Parameters can be configured in launch files:

```python
Node(
    package='robot_control_nodes',
    executable='joint_publisher_node',
    name='joint_publisher_node',
    parameters=[
        {'param_name': 'param_value'},
        '/path/to/parameter/file.yaml'
    ],
    output='screen',
)
```

## Testing

### Unit Tests

Create tests in the `test/` directory:

```python
import unittest
import rclpy
from robot_control_nodes.nodes.joint_publisher_node import JointPublisherNode

class TestJointPublisherNode(unittest.TestCase):
    def test_node_creation(self):
        rclpy.init()
        try:
            node = JointPublisherNode()
            self.assertIsNotNone(node)
        finally:
            rclpy.shutdown()
```

### Running Tests

```bash
# Run all tests
colcon test

# Run specific tests
colcon test --packages-select robot_control_nodes
```

## Debugging

### Checking Active Topics

```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /joint_states sensor_msgs/msg/JointState

# Check topic info
ros2 topic info /joint_states
```

### Checking Services and Actions

```bash
# List services
ros2 service list

# Call a service
ros2 service call /trigger_gesture robot_control_interfaces/srv/GestureTrigger '{gesture_name: "wave"}'

# List actions
ros2 action list
```