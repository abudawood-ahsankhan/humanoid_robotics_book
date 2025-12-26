# Interface Contracts: Vision-Language-Action (VLA) System

## Overview

This document defines the interface contracts between different components of the Vision-Language-Action system. These contracts ensure proper integration between the perception, reasoning, and execution layers.

## 1. Voice Command Interface

### 1.1 Voice Command Node
**Package**: `vla_integration`
**Node**: `voice_command_node`

**Subscribed Topics**:
- `/audio_input` (sensor_msgs/AudioData)
  - Raw audio data from microphone
  - Rate: Variable, typically 16kHz

**Published Topics**:
- `/vla/command_text` (std_msgs/String)
  - Transcribed text from voice command
  - Rate: On voice detection

**Services**:
- `/vla/process_voice` (vla_msgs/srv/ProcessVoice)
  - Input: audio data
  - Output: transcribed text with confidence score

### 1.2 Whisper Processing Node
**Package**: `vla_integration`
**Node**: `whisper_node`

**Subscribed Topics**:
- `/vla/audio_chunk` (std_msgs/String)
  - Audio chunks for processing
  - Rate: Variable based on voice activity

**Published Topics**:
- `/vla/transcription` (vla_msgs/msg/Transcription)
  - Contains: text, confidence, timestamp
  - Rate: On successful transcription

## 2. LLM Reasoning Interface

### 2.1 Reasoning Node
**Package**: `vla_reasoning`
**Node**: `reasoning_node`

**Subscribed Topics**:
- `/vla/command_text` (std_msgs/String)
  - Natural language commands from voice processing
  - Rate: On command received

**Published Topics**:
- `/vla/action_plan` (vla_msgs/msg/ActionPlan)
  - Structured action plan in JSON format
  - Rate: On successful plan generation

**Services**:
- `/vla/generate_plan` (vla_msgs/srv/GeneratePlan)
  - Input: command text, context
  - Output: action plan with confidence

### 2.2 LLM Interface Node
**Package**: `vla_reasoning`
**Node**: `llm_interface_node`

**Subscribed Topics**:
- `/vla/plan_request` (vla_msgs/msg/PlanRequest)
  - Request for action plan generation
  - Rate: On request

**Published Topics**:
- `/vla/plan_response` (vla_msgs/msg/PlanResponse)
  - Generated action plan with metadata
  - Rate: On response

## 3. Perception Interface

### 3.1 Perception Node
**Package**: `vla_perception`
**Node**: `perception_node`

**Subscribed Topics**:
- `/vla/query_objects` (vla_msgs/msg/ObjectQuery)
  - Object detection queries from reasoning
  - Rate: On demand

- `/camera/image_raw` (sensor_msgs/Image)
  - Raw camera images
  - Rate: 30Hz

- `/camera/camera_info` (sensor_msgs/CameraInfo)
  - Camera calibration data
  - Rate: As needed

**Published Topics**:
- `/vla/detected_objects` (vla_msgs/msg/ObjectList)
  - List of detected objects with properties
  - Rate: On detection

- `/vla/scene_description` (vla_msgs/msg/SceneDescription)
  - High-level scene understanding
  - Rate: On request

**Services**:
- `/vla/detect_objects` (vla_msgs/srv/DetectObjects)
  - Input: object types to detect
  - Output: list of detected objects with poses

### 3.2 Object Recognition Node
**Package**: `vla_perception`
**Node**: `object_recognition_node`

**Subscribed Topics**:
- `/vla/recognition_request` (vla_msgs/msg/RecognitionRequest)
  - Recognition requests with context
  - Rate: On request

**Published Topics**:
- `/vla/recognition_result` (vla_msgs/msg/RecognitionResult)
  - Recognition results with confidence
  - Rate: On completion

## 4. Execution Interface

### 4.1 Execution Node
**Package**: `vla_execution`
**Node**: `execution_node`

**Subscribed Topics**:
- `/vla/action_plan` (vla_msgs/msg/ActionPlan)
  - Action plans from reasoning layer
  - Rate: On plan received

- `/vla/execution_status` (vla_msgs/msg/ExecutionStatus)
  - Status updates during execution
  - Rate: Periodic updates

**Published Topics**:
- `/vla/execution_feedback` (vla_msgs/msg/ExecutionFeedback)
  - Detailed feedback on action execution
  - Rate: On feedback needed

**Actions**:
- `/vla_execute` (vla_msgs/action/ExecuteAction)
  - Input: action type, parameters
  - Feedback: execution progress
  - Result: success/failure with details

### 4.2 Navigation Interface
**Package**: `vla_execution`
**Node**: `navigation_node`

**Subscribed Topics**:
- `/vla/nav_goal` (geometry_msgs/PoseStamped)
  - Navigation goals from action plans
  - Rate: On goal received

**Published Topics**:
- `/vla/nav_feedback` (nav_msgs/Path)
  - Navigation progress feedback
  - Rate: Continuous during navigation

**Actions**:
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose)
  - Standard navigation action interface
  - Compatible with Nav2 system

### 4.3 Manipulation Interface
**Package**: `vla_execution`
**Node**: `manipulation_node`

**Subscribed Topics**:
- `/vla/grasp_request` (vla_msgs/msg/GraspRequest)
  - Grasp requests with object information
  - Rate: On request

**Published Topics**:
- `/vla/grasp_result` (vla_msgs/msg/GraspResult)
  - Grasp success/failure results
  - Rate: On completion

**Actions**:
- `/grasp_object` (vla_msgs/action/GraspAction)
  - Input: object pose, grasp parameters
  - Feedback: grasp progress
  - Result: grasp success with details

## 5. Safety Interface

### 5.1 Safety Monitor Node
**Package**: `vla_execution`
**Node**: `safety_monitor_node`

**Subscribed Topics**:
- All action execution topics
- Robot state topics (joint states, etc.)
- Sensor topics (lidar, etc.)

**Published Topics**:
- `/vla/safety_status` (vla_msgs/msg/SafetyStatus)
  - Safety status and violations
  - Rate: Continuous monitoring

- `/emergency_stop` (std_msgs/Bool)
  - Emergency stop commands
  - Rate: On safety violation

**Services**:
- `/vla/check_safety` (vla_msgs/srv/CheckSafety)
  - Input: proposed action
  - Output: safety assessment

## 6. Message Definitions

### 6.1 ActionPlan Message
```
string command_text
string plan_id
int32 plan_version
Action[] actions
float32 confidence
string[] safety_constraints
builtin_interfaces/Time created_time
```

### 6.2 Action Message
```
string action_id
string action_type  # navigation, manipulation, perception, communication
string description
KeyValue[] parameters
float32 priority
builtin_interfaces/Time estimated_duration
```

### 6.3 Object Message
```
string object_id
string class_name
float32 confidence
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions
string[] properties
```

### 6.4 ExecutionStatus Message
```
string plan_id
string action_id
string status  # pending, in_progress, completed, failed
float32 progress  # 0.0 to 1.0
string[] feedback
builtin_interfaces/Time start_time
builtin_interfaces/Time estimated_completion
```

## 7. Service Definitions

### 7.1 GeneratePlan Service
**Request**:
```
string command
string[] context
string[] constraints
```

**Response**:
```
bool success
ActionPlan plan
string error_message
float32 confidence
```

### 7.2 DetectObjects Service
**Request**:
```
string[] object_types
string camera_frame
float32 confidence_threshold
```

**Response**:
```
bool success
Object[] objects
string error_message
float32 processing_time
```

## 8. Quality of Service (QoS) Settings

### 8.1 Default QoS
- **Reliability**: Best effort for sensor data, Reliable for commands
- **Durability**: Volatile for streaming data, Transient local for configuration
- **History**: Keep last N samples (N varies by message type)

### 8.2 Critical Interface QoS
- **Action execution**: Reliable, transient local
- **Safety messages**: Reliable, transient local
- **Emergency stop**: Reliable, transient local, highest priority

## 9. Error Handling Contracts

### 9.1 Error Response Format
All services must respond with consistent error handling:
```
bool success
string error_code
string error_message
# Additional response fields specific to service
```

### 9.2 Error Codes
- `E_OK`: Success
- `E_TIMEOUT`: Operation timed out
- `E_INVALID_INPUT`: Invalid parameters provided
- `E_UNAVAILABLE`: Required service/resource not available
- `E_SAFETY_VIOLATION`: Safety constraint violation
- `E_EXECUTION_FAILED`: Action execution failed

## 10. Performance Guarantees

### 10.1 Latency Requirements
- Voice transcription: <500ms
- Plan generation: <2000ms
- Object detection: <200ms
- Action execution: <100ms for simple actions

### 10.2 Throughput Requirements
- Support 10 concurrent action plans
- Handle 30 voice commands per minute
- Process 30Hz sensor streams without loss

This contract ensures consistent integration between all VLA system components while maintaining flexibility for future enhancements.