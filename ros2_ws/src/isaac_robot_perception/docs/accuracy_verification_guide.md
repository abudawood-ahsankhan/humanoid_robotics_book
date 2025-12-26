# Isaac ROS Perception Pipeline Accuracy Verification Guide

## Overview

This guide provides instructions for verifying the accuracy of the Isaac ROS perception pipeline using ground truth data from Isaac Sim. The accuracy verification process compares the perception pipeline outputs against known ground truth to assess detection, classification, and localization accuracy.

## Prerequisites

- Isaac Sim running with humanoid robot model
- Isaac ROS perception pipeline running
- Ground truth topics available from Isaac Sim:
  - `/ground_truth/objects` - Ground truth object detections
  - `/ground_truth/poses` - Ground truth poses for localization
  - `/sensors/camera/image_raw` - Camera images
  - `/perception/objects` - Perception pipeline detections

## Accuracy Metrics

The verification process evaluates:

1. **Detection Accuracy**:
   - Precision: Ratio of correct detections to total detections
   - Recall: Ratio of correct detections to total ground truth objects
   - F1-Score: Harmonic mean of precision and recall
   - mAP: Mean Average Precision across all object classes

2. **Localization Accuracy**:
   - Position error: Distance between detected and ground truth positions
   - IOU: Intersection over Union of bounding boxes
   - Classification accuracy: Correct class assignment rate

## Running the Accuracy Verification

### 1. Prepare the System

Ensure all required nodes are running:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch the perception pipeline
ros2 launch isaac_robot_perception perception_pipeline.launch.py

# Ensure Isaac Sim is running with ground truth enabled
```

### 2. Run the Accuracy Verification

```bash
# Run the accuracy verification (default 60 seconds)
python3 src/isaac_robot_perception/test/accuracy_verification.py
```

### 3. Custom Verification Parameters

You can customize the verification parameters using ROS parameters:

```bash
# Run with custom test duration (120 seconds)
python3 src/isaac_robot_perception/test/accuracy_verification.py --ros-args -p test_duration:=120.0

# Run with custom accuracy thresholds
python3 src/isaac_robot_perception/test/accuracy_verification.py --ros-args \
  -p iou_threshold:=0.7 \
  -p confidence_threshold:=0.8 \
  -p position_tolerance:=0.05
```

## Accuracy Requirements

### Detection Requirements

The perception pipeline should achieve:

- **Precision**: ≥ 70% (low false positive rate)
- **Recall**: ≥ 70% (low false negative rate)
- **F1-Score**: ≥ 70% (balanced precision and recall)
- **mAP**: ≥ 60% (mean Average Precision)

### Localization Requirements

- **Position Error**: ≤ 10% of image size (for 640x480 image, ≤ 64 pixels)
- **IOU**: ≥ 50% (Intersection over Union threshold)
- **Classification Accuracy**: ≥ 80% (correct class assignments)

## Interpreting Results

The accuracy verification report includes:

1. **Overall Metrics**:
   - Overall accuracy percentage
   - Precision, recall, and F1-score
   - Mean Average Precision (mAP)

2. **Detection Metrics**:
   - True Positives (TP): Correctly detected objects
   - False Positives (FP): Incorrectly detected objects
   - False Negatives (FN): Missed ground truth objects

3. **Localization Metrics**:
   - Mean position error
   - Median position error
   - Maximum position error
   - Percentage of detections within tolerance

4. **Pass/Fail Criteria**:
   - Whether each metric meets the requirements
   - Overall pass/fail assessment

## Troubleshooting Accuracy Issues

### Low Detection Precision

**Symptoms**: High false positive rate

**Solutions**:
- Increase confidence threshold in object detection model
- Improve background filtering
- Use more specific object models
- Reduce false positive sources in environment

### Low Detection Recall

**Symptoms**: High false negative rate

**Solutions**:
- Lower confidence threshold (carefully to avoid precision loss)
- Improve sensor coverage
- Use more sensitive detection models
- Optimize preprocessing steps

### High Localization Error

**Symptoms**: Poor position accuracy

**Solutions**:
- Calibrate camera intrinsic and extrinsic parameters
- Improve detection bounding box accuracy
- Use sub-pixel localization techniques
- Verify sensor mounting positions

### Poor Classification Accuracy

**Symptoms**: Incorrect object class assignments

**Solutions**:
- Retrain models with more diverse training data
- Improve feature extraction
- Use ensemble methods
- Verify ground truth data quality

## Accuracy Optimization Tips

### 1. Model Optimization
- Use appropriate model sizes for your accuracy requirements
- Fine-tune models with domain-specific data
- Apply transfer learning techniques
- Optimize model hyperparameters

### 2. Data Quality
- Ensure high-quality ground truth data
- Use diverse training scenarios
- Include edge cases in training data
- Apply data augmentation techniques

### 3. Sensor Calibration
- Properly calibrate all sensors
- Verify extrinsic calibration between sensors
- Apply temporal synchronization
- Use sensor fusion for improved accuracy

## Expected Results

A well-tuned Isaac ROS perception pipeline should achieve:

- ✓ Precision ≥ 70%
- ✓ Recall ≥ 70%
- ✓ F1-Score ≥ 70%
- ✓ mAP ≥ 60%
- ✓ Position accuracy within tolerance
- ✓ Classification accuracy ≥ 80%

## Next Steps

After successful accuracy verification:

1. **Validate in Different Scenarios**: Test with various lighting, environments, and object configurations
2. **Robustness Testing**: Test under challenging conditions (occlusions, clutter, etc.)
3. **Performance Trade-offs**: Balance accuracy vs. speed requirements
4. **Model Retraining**: Improve models based on identified weaknesses
5. **Documentation**: Record accuracy characteristics for system design