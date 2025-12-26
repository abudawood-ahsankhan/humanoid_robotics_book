# Robot Visualization Package

This package provides Unity-based visualization for the humanoid robot simulation.

## Overview

The robot_visualization package includes:
- Unity scene for robot visualization
- C# scripts for ROS 2 communication
- Robot model visualization and control

## Features

- High-fidelity rendering of humanoid robot
- Joint position control and animation
- ROS 2 communication via ROS#
- Real-time synchronization with Gazebo simulation

## Components

- `RobotController.cs` - Controls robot joint positions
- `ROSConnector.cs` - Handles ROS 2 communication
- `VisualizationManager.cs` - Manages visualization state

## Setup

1. Install Unity 2021.3 or later
2. Import ROS# package for Unity
3. Import URDF Importer package for Unity
4. Import the humanoid robot model

## Usage

1. Start the Gazebo simulation with the robot
2. Launch the Unity scene
3. Configure ROS connection parameters
4. The robot in Unity will synchronize with the Gazebo simulation

## Topics

- `/unity_visualization/robot_state` - Robot joint states from ROS 2
- `/unity_visualization/commands` - Commands to Unity visualization