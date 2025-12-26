---
sidebar_position: 6
---

# Python Integration with ROS 2

## Overview

Python integration in ROS 2 is achieved through the rclpy package, which provides Python bindings for ROS 2. This enables developers to create ROS 2 nodes, publishers, subscribers, services, and actions using Python.

## rclpy Fundamentals

rclpy is the Python client library for ROS 2 that provides access to the ROS 2 middleware. It allows Python developers to:

- Create ROS 2 nodes
- Publish and subscribe to topics
- Create and use services
- Implement and use actions
- Manage parameters
- Handle time and timers

## Basic Node Structure

A basic ROS 2 Python node follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, services, etc.

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Publisher

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
```

### Subscriber

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## Actions

Actions are used for long-running tasks that provide feedback:

```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Parameter Management

Nodes can declare and use parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')

        # Get parameter value
        my_param = self.get_parameter('my_parameter').value
```

## Best Practices

- Always call `rclpy.init()` before creating nodes
- Use try-finally blocks to properly destroy nodes
- Follow ROS 2 naming conventions for topics, services, and actions
- Use appropriate QoS settings for different communication patterns
- Implement proper error handling and logging
- Use launch files for complex node orchestration
- Separate business logic from ROS 2 infrastructure code

## In Our Implementation

In our humanoid robotics implementation, we created several nodes using these patterns:

### Joint State Publisher

```python
class JointPublisherNode(Node):
    def __init__(self):
        super().__init__('joint_publisher_node')

        # Create publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish at regular intervals
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names and positions
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        self.publisher_.publish(msg)
```

### Joint Command Subscriber

```python
class JointSubscriberNode(Node):
    def __init__(self):
        super().__init__('joint_subscriber_node')

        # Create subscriber for joint commands
        self.subscription = self.create_subscription(
            JointCommand,
            'joint_command',
            self.joint_command_callback,
            10
        )

    def joint_command_callback(self, msg):
        """Handle incoming joint commands"""
        if msg.joint_name in self.joint_names:
            idx = self.joint_names.index(msg.joint_name)
            self.joint_positions[idx] = msg.target_position
            # Update other joint parameters...
```

### Gesture Service

```python
class GestureServiceNode(Node):
    def __init__(self):
        super().__init__('gesture_service_node')

        # Create service
        self.srv = self.create_service(
            GestureTrigger,
            'trigger_gesture',
            self.trigger_gesture_callback
        )

    def trigger_gesture_callback(self, request, response):
        gesture_name = request.gesture_name.lower()

        if gesture_name in self.gestures:
            self.gestures[gesture_name]()
            response.success = True
            response.message = f'Gesture {gesture_name} completed successfully'
        else:
            response.success = False
            response.message = f'Unknown gesture: {gesture_name}'

        return response
```

### Movement Action

```python
class MovementActionNode(Node):
    def __init__(self):
        super().__init__('movement_action_node')

        # Create action server
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def execute_callback(self, goal_handle):
        """Execute the movement action"""
        target_pose = goal_handle.request.target_pose

        # Publish feedback periodically
        feedback_msg = MoveRobot.Feedback()
        feedback_msg.distance_to_goal = 1.0
        feedback_msg.current_status = 'Moving to target pose'

        # Simulate movement by updating joint positions
        for step in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveRobot.Result()
                result.success = False
                return result

            # Update joint positions to simulate movement
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep between steps (in real implementation, use proper timing)

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.success = True
        return result
```

## Launch Files

Launch files allow you to start multiple nodes at once:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control_nodes',
            executable='joint_publisher_node',
            name='joint_publisher_node',
            output='screen',
        ),
        Node(
            package='robot_control_nodes',
            executable='joint_subscriber_node',
            name='joint_subscriber_node',
            output='screen',
        )
    ])
```