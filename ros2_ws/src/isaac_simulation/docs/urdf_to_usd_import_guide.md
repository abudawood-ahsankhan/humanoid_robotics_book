# Importing Humanoid Robot URDF to Isaac Sim USD Format

## Overview

This guide provides instructions for converting the humanoid robot URDF to USD format for use in Isaac Sim. The process involves using Omniverse's built-in URDF import functionality.

## Prerequisites

- NVIDIA Isaac Sim installed and running
- Omniverse Create or Isaac Sim application
- Humanoid robot URDF file (humanoid_robot.urdf) located at:
  `E:\Quarter_4\Hackathon_1\humanoid-robotics-book\ros2_ws\src\isaac_simulation\worlds\humanoid_robot.urdf`

## Step-by-Step Instructions

### 1. Launch Isaac Sim
- Start Isaac Sim application
- Open a new stage or load an existing scene

### 2. Import URDF
- Go to `File` → `Import` → `URDF`
- Navigate to the URDF file location:
  `E:\Quarter_4\Hackathon_1\humanoid-robotics-book\ros2_ws\src\isaac_simulation\worlds\humanoid_robot.urdf`
- Select the file and click `Open`

### 3. Configure Import Settings
- In the import dialog:
  - Check "Merge Fixed Joints" if desired
  - Set appropriate scale factor (typically 1.0 for meters)
  - Enable "Import Inertia" to maintain physics properties
  - Enable "Import Collision" to include collision geometries
  - Select "Import as USD" format

### 4. Adjust Robot Position
- After import, position the robot appropriately in the scene
- The robot should be placed with base_link at ground level
- Ensure the robot is not intersecting with the ground plane

### 5. Verify Import
- Check that all links and joints are properly imported
- Verify that visual and collision geometries are present
- Test joint movements to ensure proper kinematics
- Check that sensors (IMU, camera, LiDAR) are positioned correctly

### 6. Export as USD
- After verification, save the robot as a USD file
- Go to `File` → `Save As`
- Choose USD format
- Save to desired location (e.g., in the worlds directory)

## Isaac Sim URDF Import Extension

Isaac Sim includes a URDF import extension that provides additional functionality:

1. Enable the extension:
   - Go to `Window` → `Extensions`
   - Search for "URDF Importer"
   - Enable the extension

2. Use the extension:
   - The extension provides a dedicated UI for URDF import
   - Offers more control over import parameters
   - Supports additional URDF features

## Troubleshooting

- **Missing Materials**: If materials don't import correctly, manually assign materials in Isaac Sim
- **Joint Issues**: Verify joint limits and types match the original URDF
- **Physics Problems**: Check that inertial properties are correctly imported
- **Scale Issues**: Ensure the robot is at the correct scale (1 meter = 1 meter in Isaac Sim)

## Post-Import Configuration

After importing, additional configuration may be needed:

1. **Physics Configuration**:
   - Add rigid body components to links
   - Configure joint drives for actuation
   - Set appropriate physics materials

2. **Sensor Configuration**:
   - Add Isaac Sim sensor components to sensor links
   - Configure sensor parameters (range, resolution, etc.)
   - Verify sensor data is being published to ROS topics

3. **Articulation Chain**:
   - Create articulation chains for robot control
   - Configure drive properties for each joint
   - Set up position and velocity control

## Next Steps

After successfully importing the URDF to USD format:

1. Test the robot in various Isaac Sim environments
2. Verify sensor data publication to ROS topics
3. Configure Nav2 integration for navigation tasks
4. Test perception pipeline with the imported robot