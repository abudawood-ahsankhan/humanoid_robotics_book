# Physics Configuration Guide for Humanoid Robot

## Overview

This guide provides instructions for configuring physics properties for the humanoid robot in Isaac Sim. Proper physics configuration is essential for realistic simulation and stable bipedal locomotion.

## Prerequisites

- NVIDIA Isaac Sim installed and running
- Humanoid robot model imported into Isaac Sim
- Understanding of rigid body dynamics and joint constraints

## Configuration Steps

### 1. Enable Physics Extensions

Before configuring physics:

1. Open Isaac Sim
2. Go to `Window` → `Extensions`
3. Enable the following extensions:
   - `omni.physics` (Physics simulation)
   - `omni.isaac.dynamic_control` (Articulation control)
   - `omni.isaac.core_nodes` (Core simulation nodes)

### 2. Configure Rigid Body Properties

For each robot link, configure rigid body properties:

1. Select the robot model in the Stage
2. For each link, add a RigidBody component:
   - Right-click on the link in the Stage
   - Select `Add` → `Physics` → `RigidBody`
   - Configure mass, friction, and damping properties

3. Set appropriate mass values based on the physics_config.yaml:
   - Base link (torso): 10.0 kg
   - Head: 2.0 kg
   - Leg components: 1.0-3.0 kg
   - Arm components: 0.5-1.5 kg

4. Configure material properties:
   - Static friction: 0.4-0.8 (higher for feet)
   - Dynamic friction: 0.3-0.7
   - Restitution (bounciness): 0.05-0.1

### 3. Configure Joint Properties

For each joint in the robot:

1. Select the joint in the Stage
2. Add a Joint component if not already present
3. Configure joint limits and drive properties:
   - Set appropriate joint limits based on humanoid anatomy
   - Configure stiffness and damping for stable control
   - Set maximum force and velocity limits

4. For bipedal locomotion, pay special attention to:
   - Hip joints: Allow sufficient range for walking
   - Knee joints: Positive limits for forward bending
   - Ankle joints: Limited range for stability

### 4. Set Up Articulation

Configure the robot as an articulation:

1. Select the base link of the robot
2. Add an ArticulationRoot component:
   - Right-click → `Add` → `Physics` → `ArticulationRoot`
   - This allows coordinated control of all joints

2. Configure drive properties for each joint:
   - Position drives for precise control
   - Appropriate stiffness and damping values
   - Maximum force limits to prevent damage

### 5. Apply Physics Configuration

Use the physics_config.yaml file to set up advanced physics properties:

1. **Global Physics Settings**:
   - Set gravity to standard Earth value (-9.81 m/s²)
   - Configure physics update frequency (60 Hz recommended)
   - Adjust solver iterations for stability

2. **Material Properties**:
   - Assign different materials to different robot parts
   - Higher friction for feet for better grip
   - Appropriate density for realistic mass distribution

### 6. Configure Ground Plane

For stable simulation:

1. Add a ground plane to the scene:
   - Create → Primitive → Plane
   - Position at appropriate height (typically z=0)

2. Add physics properties to the ground:
   - Add Collision component
   - Set appropriate friction values
   - Ensure it's static (infinite mass)

### 7. Test Physics Configuration

After configuring physics:

1. Run a basic simulation to test:
   - Robot falls and stabilizes properly
   - Joints behave as expected
   - No unexpected collisions or interpenetrations

2. Test joint limits:
   - Move joints through their range of motion
   - Verify limits prevent dangerous positions

3. Test stability:
   - Check if robot remains stable when standing
   - Verify appropriate friction prevents sliding

## Isaac Sim Physics Extensions

### PhysX Configuration

For PhysX-based physics simulation:

1. Go to `Window` → `Physics` → `PhysX Settings`
2. Configure:
   - Solver type (PGS recommended for articulations)
   - Iteration counts for stability
   - Collision detection settings

### Dynamic Control

For advanced articulation control:

1. Use the Dynamic Control API for precise joint control
2. Configure joint drives for position, velocity, or effort control
3. Set up coordinated movement patterns for bipedal locomotion

## Verification Steps

After configuring physics:

1. **Stability Test**:
   - Robot should remain stable when standing
   - No unexpected oscillations or movements
   - Proper response to gravity

2. **Joint Range Test**:
   - All joints move within expected ranges
   - Joint limits prevent dangerous positions
   - Smooth movement without sticking

3. **Collision Test**:
   - Robot doesn't collide with itself unexpectedly
   - Proper interaction with environment
   - Realistic contact responses

## Troubleshooting

- **Robot Falls Through Ground**: Check collision components and ground plane settings
- **Joints Too Stiff**: Reduce stiffness values in joint configuration
- **Unstable Simulation**: Increase solver iterations or reduce time step
- **Robot Slides Excessively**: Increase friction values on feet
- **Joints Don't Move Properly**: Check joint limits and drive configuration

## Next Steps

After configuring physics properties:

1. Test bipedal locomotion with the physics-enabled robot
2. Validate physics behavior in various environments
3. Tune parameters based on locomotion performance
4. Document final physics settings for reproducibility