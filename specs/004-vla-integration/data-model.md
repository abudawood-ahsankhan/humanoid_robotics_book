# Data Model: Vision-Language-Action (VLA) System

## Overview
This document describes the key data structures and relationships for the Vision-Language-Action system that integrates voice commands, LLM-based reasoning, visual perception, and robotic action execution.

## Core Data Structures

### 1. Voice Command Structure
```
VoiceCommand {
  id: UUID
  audio_data: Binary
  transcribed_text: String
  timestamp: DateTime
  confidence: Float (0.0-1.0)
  user_intent: String
  context: JSON
}
```

### 2. Structured Action Plan
```
ActionPlan {
  id: UUID
  original_command: String
  plan_version: String
  tasks: [Task]
  priority: Enum(HIGH, MEDIUM, LOW)
  estimated_duration: Float (seconds)
  safety_constraints: [SafetyConstraint]
  created_timestamp: DateTime
  status: Enum(PENDING, IN_PROGRESS, COMPLETED, FAILED)
}
```

### 3. Task Definition
```
Task {
  id: UUID
  type: Enum(NAVIGATION, MANIPULATION, PERCEPTION, COMMUNICATION)
  description: String
  parameters: JSON
  dependencies: [UUID]
  success_criteria: String
  timeout: Float (seconds)
}
```

### 4. Perception Data
```
PerceptionData {
  id: UUID
  sensor_type: Enum(CAMERA, LIDAR, IMU, OTHER)
  timestamp: DateTime
  frame_id: String
  data: Binary (image/pointcloud)
  processed_objects: [DetectedObject]
  confidence_threshold: Float
}
```

### 5. Detected Object
```
DetectedObject {
  id: UUID
  class_name: String
  bounding_box: {
    x: Float
    y: Float
    width: Float
    height: Float
  }
  position_3d: {
    x: Float
    y: Float
    z: Float
  }
  confidence: Float (0.0-1.0)
  properties: JSON
}
```

### 6. Safety Constraint
```
SafetyConstraint {
  id: UUID
  constraint_type: Enum(VELOCITY_LIMIT, FORCE_LIMIT, DISTANCE_LIMIT, AREA_RESTRICTION)
  parameters: JSON
  severity: Enum(INFO, WARNING, ERROR, CRITICAL)
  activation_conditions: JSON
}
```

## Relationships

- VoiceCommand → ActionPlan (1:1) - Each voice command generates one action plan
- ActionPlan → Task (1:N) - Each action plan contains multiple tasks
- Task → PerceptionData (N:N) - Tasks may require perception data
- PerceptionData → DetectedObject (1:N) - Perception data contains multiple objects
- ActionPlan → SafetyConstraint (1:N) - Action plans have multiple safety constraints

## Data Flow

1. VoiceCommand is processed by speech recognition
2. LLM generates ActionPlan based on transcribed text
3. ActionPlan is decomposed into Tasks
4. Tasks trigger PerceptionData collection
5. PerceptionData is processed to identify DetectedObjects
6. Tasks are executed with SafetyConstraints applied
7. Execution status is updated in ActionPlan

## Serialization Formats

- Action plans: JSON format with schema validation
- Perception data: ROS 2 message formats (sensor_msgs, vision_msgs)
- Voice data: WAV format for processing
- Object detection: Custom message format based on vision_msgs

## Constraints and Validation

- Action plan must not exceed maximum task count (default: 50)
- Each task must have defined success criteria
- Safety constraints must be validated before task execution
- Perception data must meet minimum confidence thresholds
- All timestamps must be synchronized across components