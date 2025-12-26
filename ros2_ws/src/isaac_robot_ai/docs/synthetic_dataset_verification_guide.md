# Synthetic Dataset Generation Verification Guide

## Overview

This guide provides instructions for verifying that the synthetic dataset generation pipeline is working correctly and producing valid training data for AI models.

## Prerequisites

- Isaac Sim with humanoid robot model
- Isaac Sim-ROS bridge configured
- Synthetic dataset generator node running
- Isaac ROS perception packages installed

## Verification Steps

### 1. Pre-Verification Setup

Before verifying synthetic dataset generation:

1. **Launch Required Components**:
   ```bash
   # Source ROS 2 environment
   source /opt/ros/humble/setup.bash
   cd ~/humanoid_ws
   source install/setup.bash

   # Launch Isaac Sim with the humanoid robot
   # Launch the Isaac Sim-ROS bridge
   ros2 launch isaac_simulation isaac_sim_bridge.launch.py
   ```

2. **Verify Sensor Data**:
   ```bash
   # Check that sensor data is being published
   ros2 topic echo /sensors/camera/image_raw --field header.stamp -1
   ros2 topic echo /perception/objects --field detections -1
   ```

### 2. Run Automated Verification

Execute the automated verification script:

```bash
# Run the verification test
python3 src/isaac_robot_ai/test/test_synthetic_dataset_generation.py
```

### 3. Manual Verification Steps

In addition to automated tests, perform manual verification:

1. **Check Dataset Directory Structure**:
   ```bash
   # Verify directory structure
   ls -la /tmp/synthetic_dataset/
   # Should contain: images/, labels/, camera_info/, dataset_info.json
   ```

2. **Verify Image Quality**:
   ```bash
   # Check image count and properties
   ls -la /tmp/synthetic_dataset/images/ | wc -l
   # View sample images
   eog /tmp/synthetic_dataset/images/sample_*.png  # or your preferred image viewer
   ```

3. **Verify Label Format**:
   ```bash
   # Check label format
   head -20 /tmp/synthetic_dataset/labels/sample_0000_detections.json
   ```

4. **Check Camera Info**:
   ```bash
   # Verify camera parameters
   cat /tmp/synthetic_dataset/camera_info/sample_0000_camera_info.json
   ```

### 4. Dataset Quality Assessment

Evaluate the quality of generated datasets:

1. **Image Quality**:
   - Check for realistic lighting and shadows
   - Verify absence of visual artifacts
   - Confirm proper resolution and color depth

2. **Label Accuracy**:
   - Verify bounding boxes align with objects
   - Check that object classes are correctly labeled
   - Ensure no missing or incorrect annotations

3. **Consistency**:
   - Verify consistent naming conventions
   - Check that timestamps are properly synchronized
   - Confirm matching between images, labels, and camera info

### 5. Performance Verification

Test the generation performance:

1. **Capture Rate**:
   ```bash
   # Monitor capture rate
   ros2 run topic_tools relay /sensors/camera/image_raw /dev/null
   ```

2. **Storage Usage**:
   ```bash
   # Check dataset size
   du -sh /tmp/synthetic_dataset/
   ```

3. **Memory Usage**:
   ```bash
   # Monitor memory usage during generation
   htop  # or your preferred monitoring tool
   ```

## Expected Results

After successful synthetic dataset generation verification:

1. ✓ Dataset directory contains expected subdirectories
2. ✓ Images are properly formatted and realistic
3. ✓ Labels are correctly formatted and accurate
4. ✓ Camera info files contain valid parameters
5. ✓ File naming follows consistent pattern
6. ✓ Dataset info file contains metadata

## Troubleshooting

### No Images Being Generated
- Check that camera sensor is publishing data
- Verify Isaac Sim-ROS bridge connection
- Confirm camera topic is correctly mapped

### Missing Labels
- Verify perception pipeline is running
- Check that object detection is working
- Confirm detection topic is correctly configured

### Incorrect File Formats
- Check file extensions match expected format
- Verify image encoding is correct
- Confirm JSON files are properly formatted

### Performance Issues
- Reduce capture frequency if system is overloaded
- Check available disk space
- Monitor CPU and GPU utilization

## Quality Metrics

### Dataset Quality Indicators
- **Image Quality**: Clear, properly exposed images with realistic lighting
- **Label Accuracy**: Bounding boxes accurately surround objects
- **Temporal Consistency**: Sequential frames maintain consistent object tracking
- **Variety**: Diverse scenarios, lighting conditions, and object configurations

### Performance Metrics
- **Capture Rate**: Should match configured frequency
- **Storage Efficiency**: Reasonable size per image
- **Processing Time**: Minimal delay between capture and save

## Validation Commands

Run these commands to validate dataset generation:

```bash
# Check for missing files
python3 -c "
import os
ds_path = '/tmp/synthetic_dataset'
for subdir in ['images', 'labels', 'camera_info']:
    files = os.listdir(os.path.join(ds_path, subdir))
    print(f'{subdir}: {len(files)} files')
"

# Validate JSON files
find /tmp/synthetic_dataset -name '*.json' -exec python3 -m json.tool {} \; > /dev/null && echo 'All JSON files valid' || echo 'Invalid JSON files found'

# Check image formats
find /tmp/synthetic_dataset/images -name '*.png' -exec file {} \; | head -5
```

## Next Steps

After successful verification:

1. Generate larger datasets for AI training
2. Test dataset quality with actual AI models
3. Validate sim-to-real transfer capabilities
4. Document optimal generation parameters