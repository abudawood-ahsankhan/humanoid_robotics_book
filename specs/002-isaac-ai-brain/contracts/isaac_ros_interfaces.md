# Isaac ROS Interface Contracts: AI Robot Brain System

## Perception Pipeline Interfaces

### Sensor Input Topics
- **Topic**: `/sensors/camera/image_raw`
- **Type**: `sensor_msgs/Image`
- **Direction**: Subscriber (from Isaac Sim to perception nodes)
- **Purpose**: Raw camera image data for perception processing

- **Topic**: `/sensors/lidar/points`
- **Type**: `sensor_msgs/PointCloud2`
- **Direction**: Subscriber (from Isaac Sim to perception nodes)
- **Purpose**: LiDAR point cloud data for 3D perception

- **Topic**: `/sensors/imu/data`
- **Type**: `sensor_msgs/Imu`
- **Direction**: Subscriber (from Isaac Sim to perception nodes)
- **Purpose**: IMU data for sensor fusion and localization

### Perception Output Topics
- **Topic**: `/perception/objects`
- **Type**: `vision_msgs/Detection2DArray`
- **Direction**: Publisher (from perception nodes to navigation/AI nodes)
- **Purpose**: Detected objects with bounding boxes and labels

- **Topic**: `/perception/segmentation`
- **Type**: `sensor_msgs/Image`
- **Direction**: Publisher (from perception nodes to other nodes)
- **Purpose**: Semantic segmentation masks for environment understanding

- **Topic**: `/perception/features`
- **Type**: `sensor_msgs/PointCloud2`
- **Direction**: Publisher (from perception nodes to VSLAM)
- **Purpose**: Visual features for SLAM processing

## VSLAM Interfaces

### Map and Pose Topics
- **Topic**: `/vslam/map`
- **Type**: `nav_msgs/OccupancyGrid` or `sensor_msgs/PointCloud2`
- **Direction**: Publisher (from VSLAM nodes)
- **Purpose**: Generated environmental map for navigation

- **Topic**: `/vslam/pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Direction**: Publisher (from VSLAM nodes)
- **Purpose**: Robot pose estimate in the map frame

- **Topic**: `/vslam/tf`
- **Type**: `tf2_msgs/TFMessage`
- **Direction**: Publisher (from VSLAM nodes)
- **Purpose**: Transform between map and robot frames

## Navigation Interfaces

### Navigation Commands
- **Topic**: `/goal_pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Direction**: Subscriber (from external nodes to navigation)
- **Purpose**: Navigation goal pose input

- **Topic**: `/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Direction**: Publisher (from navigation to robot controller)
- **Purpose**: Velocity commands for robot movement

### Navigation Status
- **Topic**: `/navigation/status`
- **Type**: `action_msgs/GoalStatusArray`
- **Direction**: Publisher (from navigation nodes)
- **Purpose**: Current navigation goal status

- **Topic**: `/navigation/path`
- **Type**: `nav_msgs/Path`
- **Direction**: Publisher (from navigation nodes)
- **Purpose**: Planned path to goal

## AI Training Interfaces

### Training Data Topics
- **Topic**: `/synthetic_data/images`
- **Type**: `sensor_msgs/Image`
- **Direction**: Publisher (from Isaac Sim data generator)
- **Purpose**: Synthetic images for AI model training

- **Topic**: `/synthetic_data/labels`
- **Type**: `vision_msgs/Detection2DArray`
- **Type**: Publisher (from Isaac Sim data generator)
- **Purpose**: Ground truth labels for synthetic data

### Model Interface Services
- **Service**: `/ai/load_model`
- **Type**: `std_srvs/Trigger`
- **Purpose**: Load trained AI model into perception system

- **Service**: `/ai/train_agent`
- **Type**: `std_srvs/SetBool`
- **Purpose**: Start/stop AI agent training with parameters

## Isaac Sim Bridge Interfaces

### Simulation Control
- **Topic**: `/isaac_sim/control`
- **Type**: `std_msgs/String`
- **Direction**: Publisher (from ROS to Isaac Sim)
- **Purpose**: Simulation control commands (reset, pause, resume)

- **Topic**: `/isaac_sim/state`
- **Type**: `std_msgs/String`
- **Direction**: Publisher (from Isaac Sim to ROS)
- **Purpose**: Current simulation state information

### Robot State Interface
- **Topic**: `/isaac_sim/joint_states`
- **Type**: `sensor_msgs/JointState`
- **Direction**: Publisher (from Isaac Sim to ROS)
- **Purpose**: Joint positions, velocities, and efforts from simulation

- **Topic**: `/isaac_sim/tf`
- **Type**: `tf2_msgs/TFMessage`
- **Direction**: Publisher (from Isaac Sim to ROS)
- **Purpose**: Robot transforms from simulation