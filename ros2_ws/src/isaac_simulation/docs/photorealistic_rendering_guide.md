# Photorealistic Rendering Configuration Guide

## Overview

This guide provides instructions for configuring photorealistic rendering settings for the humanoid robot in Isaac Sim. Proper rendering configuration is essential for generating synthetic data that closely matches real-world conditions.

## Prerequisites

- NVIDIA Isaac Sim installed and running
- Humanoid robot model imported into Isaac Sim
- Omniverse Create or Isaac Sim application with rendering extensions enabled

## Configuration Steps

### 1. Enable Required Extensions

Before configuring photorealistic rendering:

1. Open Isaac Sim
2. Go to `Window` → `Extensions`
3. Enable the following extensions:
   - `omni.pbr` (Physically Based Rendering)
   - `omni.kit.renderer.core`
   - `omni.hydra.pxr`
   - `omni.kit.window.viewport`
   - `omni.isaac.range_sensor` (for sensor simulation)

### 2. Apply Material Properties

For each robot link, assign appropriate photorealistic materials:

1. Select the robot model in the Stage
2. For each link, apply materials from the Materials Library:
   - Head/Body: Skin-like material with subsurface scattering
   - Limbs: Plastic or metal materials depending on design
   - Joints: Metallic materials for realistic reflections
   - Sensors: Matte black materials to reduce reflections

3. Adjust material properties:
   - Metallic: 0.0-0.9 (0.0 for non-metallic, 0.9 for metallic)
   - Roughness: 0.1-0.9 (0.1 for smooth, 0.9 for rough)
   - Specular: 0.0-1.0 (amount of specular reflection)
   - Normal map: Apply for surface detail

### 3. Configure Lighting

Set up realistic lighting in the scene:

1. **Environment Lighting**:
   - Enable Image-Based Lighting (IBL)
   - Use high-quality HDR environment maps
   - Adjust IBL intensity to match desired lighting conditions

2. **Key Light**:
   - Add a distant light as the key light
   - Set intensity to 1000-2000 lumens
   - Position to create realistic shadows
   - Enable shadows with high resolution (2048x2048)

3. **Fill Light**:
   - Add a secondary light to fill shadows
   - Set to lower intensity (30-50% of key light)
   - Use slightly different color temperature

4. **Rim Light**:
   - Add light from behind the robot
   - Creates separation from background
   - Helps with object recognition

### 4. Configure Camera Settings

For the robot's camera sensor:

1. Select the camera in the scene
2. Adjust camera properties:
   - Enable distortion if needed for realism
   - Set appropriate field of view (90 degrees recommended)
   - Configure resolution and frame rate
   - Enable sensor noise for realistic image quality

### 5. Apply Rendering Configuration

Use the rendering_config.yaml file to set up advanced rendering features:

1. **Path Tracing Settings**:
   - Enable Path Tracing for photorealistic lighting
   - Configure denoising for faster rendering
   - Set maximum surface bounces for realistic light paths

2. **Post-Processing Effects**:
   - Enable bloom for realistic light bloom
   - Configure depth of field if needed
   - Adjust color grading for desired look

### 6. Optimize Performance

For real-time rendering during simulation:

1. **Quality Settings**:
   - Adjust rendering quality based on hardware
   - Use lower resolution for faster training data generation
   - Balance quality vs. performance requirements

2. **Viewport Settings**:
   - Use lower quality in viewport during simulation
   - Use full quality only for final rendering
   - Configure appropriate render update frequency

## Isaac Sim Rendering Extensions

### RTX Renderer Configuration

For RTX-based rendering:

1. Go to `Window` → `Renderer` → `Settings`
2. Select RTX renderer
3. Enable:
   - Denoising for faster convergence
   - Reflections for realistic mirror effects
   - Refractions for transparent materials
   - Subsurface scattering for skin-like materials

### Material Configuration

1. Open Material Library (`Window` → `Material Library`)
2. Create custom materials for robot components:
   - Use OmniPBR for photorealistic materials
   - Configure base color, metallic, and roughness
   - Add normal maps for surface detail
   - Set appropriate IOR (Index of Refraction)

## Verification Steps

After configuring rendering:

1. **Visual Inspection**:
   - Check for realistic lighting and shadows
   - Verify materials look photorealistic
   - Ensure no visual artifacts

2. **Performance Check**:
   - Monitor rendering frame rate
   - Ensure stable performance during simulation
   - Adjust settings if performance is too slow

3. **Synthetic Data Quality**:
   - Capture sample images to verify quality
   - Check for realistic sensor noise
   - Verify colors and lighting match expectations

## Troubleshooting

- **Slow Rendering**: Reduce quality settings or use viewport rendering
- **Dark Images**: Increase lighting intensity or adjust camera exposure
- **Artifacts**: Check material assignments and lighting setup
- **Memory Issues**: Reduce texture resolution or rendering quality

## Next Steps

After configuring photorealistic rendering:

1. Generate synthetic training data with the configured settings
2. Test perception pipeline with rendered images
3. Adjust rendering parameters based on perception performance
4. Document final rendering settings for reproducibility