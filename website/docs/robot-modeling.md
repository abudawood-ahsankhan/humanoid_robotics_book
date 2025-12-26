---
sidebar_position: 7
---

# Robot Modeling with URDF

## Overview

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the robot's physical and visual properties, including links, joints, inertial properties, and sensor placements.

## URDF Structure

A URDF file typically contains:

- **Links**: Rigid bodies of the robot
- **Joints**: Connections between links
- **Visual**: Visual representation for rendering
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties

## Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Child link connected via joint -->
  <link name="upper_body">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting the links -->
  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.5"/>
  </joint>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have:

- Visual properties (shape, color, material)
- Collision properties (for physics simulation)
- Inertial properties (mass, center of mass, inertia)

### Link Properties

```xml
<link name="link_name">
  <!-- Visual representation -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
      <!-- Other options: cylinder, sphere, mesh -->
    </geometry>
    <material name="material_name">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

## Joints

Joints connect links and define how they can move relative to each other. Joint types include:

- **Fixed**: No movement between links
- **Revolute**: Single-axis rotation with limits
- **Continuous**: Single-axis rotation without limits
- **Prismatic**: Single-axis translation with limits
- **Planar**: Planar motion
- **Floating**: 6-DOF motion

### Joint Properties

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Humanoid Robot Specific Considerations

When modeling humanoid robots, consider:

- **Kinematic chains**: Arms, legs, and spine as connected chains
- **Degrees of freedom**: Ensure sufficient DOF for desired movements
- **Joint limits**: Realistic limits based on human anatomy
- **Mass distribution**: Realistic mass properties for stable simulation
- **Sensor placement**: Proper placement for cameras, IMUs, etc.

## URDF for Humanoid Robots

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Main body -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>
</robot>
```

## Best Practices

- Use consistent naming conventions for links and joints
- Validate URDF files using `check_urdf` command
- Use appropriate mesh files for complex geometries
- Include proper inertial properties for stable simulation
- Consider using Xacro for complex models to avoid repetition
- Test URDF in RViz and Gazebo before finalizing

## In Our Implementation

In our humanoid robotics implementation, we created a URDF model that includes:

- Multiple body segments (torso, head, limbs)
- Proper joint definitions for realistic movement
- Visual and collision properties for simulation
- Proper inertial properties for stable physics

### Our Humanoid Robot Model

Our implementation includes a complete humanoid robot with 8 degrees of freedom:

#### Link Structure

- `base_link`: Mobile base of the robot
- `torso`: Main body of the robot
- `head`: Head with camera sensor
- `left_upper_arm`, `left_lower_arm`: Left arm segments
- `right_upper_arm`, `right_lower_arm`: Right arm segments
- `left_upper_leg`, `left_lower_leg`: Left leg segments
- `right_upper_leg`, `right_lower_leg`: Right leg segments

#### Joint Configuration

Each joint has appropriate limits and properties:

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.1 0 0.1"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

#### Visual and Collision Properties

Visual properties define how the robot appears in simulation and visualization:

```xml
<visual>
  <geometry>
    <cylinder length="0.2" radius="0.025"/>
  </geometry>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
</visual>
```

Collision properties define the collision geometry used in simulation:

```xml
<collision>
  <geometry>
    <cylinder length="0.2" radius="0.025"/>
  </geometry>
</collision>
```

#### Inertial Properties

Inertial properties are important for physics simulation:

```xml
<inertial>
  <mass value="0.5"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

#### Sensors in URDF

We've added sensors to our robot model:

```xml
<gazebo reference="head">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
    </camera>
  </sensor>
</gazebo>
```

#### Working with URDF in ROS

##### Robot State Publisher

The robot_state_publisher node reads the URDF and publishes the robot's joint states to the TF system:

```python
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
        'robot_description': open(urdf_file).read()
    }]
)
```

##### Joint State Publisher

The joint_state_publisher provides GUI controls for setting joint positions in visualization:

```python
joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen'
)
```

#### RViz Visualization

RViz is used to visualize the robot model with its current joint states. Our implementation includes a preconfigured RViz setup that shows the robot model properly.