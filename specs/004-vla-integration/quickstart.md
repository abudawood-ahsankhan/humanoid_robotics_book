# Quickstart Guide: Vision-Language-Action (VLA) System

## Overview
This guide provides instructions for setting up and using the Vision-Language-Action system that enables humanoid robots to understand voice commands, reason about tasks, and execute actions in simulation.

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 LTS
   - ROS 2 Humble Hawksbill
   - NVIDIA GPU with CUDA support (for Isaac Sim if using)
   - Python 3.8+
   - At least 16GB RAM

2. **Software Dependencies**:
   ```bash
   # Install ROS 2 dependencies
   sudo apt update
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

   # Install audio processing dependencies
   pip3 install openai-whisper
   pip3 install sounddevice
   pip3 install pyaudio

   # Install LLM dependencies
   pip3 install openai
   pip3 install transformers
   pip3 install torch

   # Install computer vision dependencies
   pip3 install opencv-python
   pip3 install numpy
   pip3 install pillow
   ```

3. **Simulation Environment**:
   - Gazebo Garden OR Isaac Sim 2023.1+
   - Humanoid robot model with ROS 2 interfaces

## Installation

1. **Clone the repository**:
   ```bash
   cd ~/humanoid_ws/src
   git clone [repository-url]/vla_integration.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/humanoid_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --packages-select vla_integration vla_perception vla_reasoning vla_execution
   source install/setup.bash
   ```

## Basic Usage

### 1. Launch the Complete VLA System

```bash
# Terminal 1: Launch simulation environment
cd ~/humanoid_ws
source install/setup.bash
ros2 launch vla_integration simulation.launch.py

# Terminal 2: Launch perception pipeline
source install/setup.bash
ros2 launch vla_perception perception_pipeline.launch.py

# Terminal 3: Launch reasoning and planning system
source install/setup.bash
ros2 launch vla_reasoning reasoning_pipeline.launch.py

# Terminal 4: Launch execution system
source install/setup.bash
ros2 launch vla_execution execution_pipeline.launch.py

# Terminal 5: Launch the VLA coordinator
source install/setup.bash
ros2 launch vla_integration vla_system.launch.py
```

### 2. Voice Command Processing

```bash
# Terminal: Start voice command interface
source install/setup.bash
ros2 run vla_integration voice_command_node
```

Then speak a command like "Move the red cube to the blue bin" into your microphone.

### 3. Direct Command Testing

You can also send commands directly:

```bash
# Send a text command directly
ros2 service call /vla/text_command vla_msgs/srv/TextCommand "{'command': 'Go to the kitchen and bring me the cup'}"
```

## Week 1: Voice-to-Text Processing

1. **Test Speech Recognition**:
   ```bash
   ros2 run vla_integration whisper_node --test
   ```

2. **Verify Command Normalization**:
   - Speak various commands to test transcription accuracy
   - Verify commands are properly normalized for LLM processing

3. **Adjust Sensitivity**:
   - Modify `config/voice_config.yaml` for optimal recognition in your environment

## Week 2: Cognitive Planning with LLMs

1. **Configure LLM Access**:
   - Set up OpenAI API key or local LLM model
   - Verify LLM connection with `ros2 run vla_reasoning llm_test_node`

2. **Test Task Decomposition**:
   - Send commands like "Clean the room" and verify they generate structured action plans
   - Check action plan format in `~/vla_logs/action_plans/`

3. **Customize Prompt Templates**:
   - Modify prompts in `config/llm_prompts/` for specific use cases

## Week 3: Vision Grounding

1. **Calibrate Perception Pipeline**:
   ```bash
   ros2 launch vla_perception calibration.launch.py
   ```

2. **Test Object Detection**:
   - Verify the system correctly identifies objects mentioned in commands
   - Test spatial reasoning capabilities

3. **Validate Scene Understanding**:
   - Test the system's ability to understand spatial relationships
   - Verify object localization accuracy

## Week 4: Action Execution

1. **Test Individual Actions**:
   ```bash
   ros2 action send_goal /vla_execute vla_msgs/action/ExecuteAction "action_type: 'navigation' parameters: {x: 1.0, y: 2.0, theta: 0.0}"
   ```

2. **Verify Safety Constraints**:
   - Test emergency stop functionality
   - Verify safety constraints are properly enforced

3. **Run Multi-Step Tasks**:
   - Execute complete tasks from voice commands
   - Monitor execution status and error handling

## Week 5: End-to-End Integration

1. **Complete Task Demonstrations**:
   - "Go to the table, pick up the red block, and place it in the box"
   - "Find the cup, bring it to me, then return to your starting position"
   - "Navigate to the kitchen and identify all the objects on the counter"

2. **Performance Testing**:
   - Measure task completion success rate
   - Evaluate response time and accuracy

3. **Troubleshooting**:
   - Address any integration issues
   - Optimize performance

## Configuration

### Voice Settings (`config/voice_config.yaml`)
- `audio_device`: Input device ID for microphone
- `sensitivity`: Threshold for voice activation
- `language`: Language for speech recognition

### LLM Settings (`config/llm_config.yaml`)
- `model`: LLM model to use (e.g., gpt-4, local model path)
- `api_key`: API key for cloud LLMs
- `temperature`: Creativity parameter (0.0-1.0)

### Perception Settings (`config/perception_config.yaml`)
- `detection_threshold`: Confidence threshold for object detection
- `tracking_enabled`: Enable object tracking
- `camera_topics`: List of camera topics to process

## Verification

1. **Check all nodes are running**:
   ```bash
   ros2 node list
   # Should include: voice_command_node, reasoning_node, perception_node, execution_node
   ```

2. **Verify topic connections**:
   ```bash
   ros2 topic list | grep vla
   ```

3. **Test system response**:
   ```bash
   # Send a simple command
   ros2 topic pub /vla/voice_command std_msgs/String "data: 'move forward 1 meter'"
   ```

## Troubleshooting

- **Voice commands not recognized**: Check microphone permissions and audio device settings
- **LLM responses are slow**: Verify internet connection and API key validity
- **Objects not detected**: Check camera calibration and lighting conditions
- **Actions fail to execute**: Verify robot control interfaces and safety systems
- **Planning fails**: Review prompt templates and LLM configuration