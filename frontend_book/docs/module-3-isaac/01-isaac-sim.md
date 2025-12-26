---
sidebar_position: 1
slug: isaac-sim
title: "Isaac Sim: Photorealistic Simulation"
sidebar_label: "Isaac Sim"
description: "Learn how to use NVIDIA Isaac Sim for photorealistic robot simulation and synthetic training data generation for humanoid perception systems."
tags:
  - isaac-sim
  - simulation
  - synthetic-data
  - domain-randomization
  - nvidia
---

# Isaac Sim: Photorealistic Simulation

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain what Isaac Sim is and why photorealistic simulation matters for humanoid robotics development
2. Install Isaac Sim via NVIDIA Omniverse and launch the application with a basic scene
3. Import a humanoid robot URDF model from Module 1 into Isaac Sim and convert it to USD format
4. Add and configure virtual sensors (RGB cameras, depth sensors, LiDAR, IMU) on the humanoid robot model
5. Run simulations with humanoid robots in realistic environments (indoor/outdoor) with accurate physics
6. Generate synthetic training datasets with annotated images (bounding boxes, segmentation masks, depth maps)
7. Apply domain randomization techniques (lighting, textures, object placement) to improve model generalization
8. Integrate Isaac Sim with ROS 2 to publish sensor data and subscribe to control commands
9. Optimize Isaac Sim performance for GPU utilization and explore cloud deployment options

## Prerequisites

:::info Before You Begin

- **Module 1 Chapter 3**: Understanding of URDF robot modeling (you'll import these models into Isaac Sim)
- **Module 2 Chapter 1**: Familiarity with basic simulation concepts from Gazebo
- **NVIDIA GPU**: RTX series or better (minimum GTX 1060), or access to cloud GPU instances
- **Disk Space**: Approximately 20GB available for Isaac Sim installation
- **Python Proficiency**: Comfortable with Python scripts for automation and data generation

:::

---

## 1. Introduction: Why Isaac Sim?

Building robot software is hard. Testing on real humanoid robots is expensive ($50k-$500k+ per robot), risky (falls damage motors), and slow (hours for setup and iteration). This is where simulation becomes essential—but not just any simulation.

### The Sim-to-Real Gap Problem

Traditional robot simulators (like Gazebo, which you learned in Module 2) provide physics-accurate simulation but often use basic rendering. When you train perception models (object detection, semantic segmentation) on sim data and deploy to real robots, they fail because:

- **Visual Mismatch**: Sim rendering looks "fake" (flat colors, simple lighting) vs real-world complexity
- **Sensor Noise**: Perfect sim sensors don't match noisy real cameras and LiDAR
- **Environment Diversity**: Limited sim environments don't capture real-world variations

**Isaac Sim solves this** by providing **photorealistic rendering** powered by NVIDIA RTX ray tracing. Your synthetic training data looks nearly identical to real camera images, dramatically reducing the sim-to-real gap.

### Isaac Sim vs Generic Simulators

| Aspect | Generic Simulators (e.g., Gazebo) | NVIDIA Isaac Sim |
|--------|-----------------------------------|------------------|
| **Rendering Quality** | Basic (OpenGL, flat shading) | Photorealistic (RTX ray tracing, PBR materials) |
| **Physics Fidelity** | Good (ODE, Bullet) | Excellent (PhysX 5, GPU-accelerated) |
| **Sensor Simulation** | Limited (basic camera, LiDAR) | Comprehensive (RGB, depth, LiDAR, IMU, force sensors) |
| **Synthetic Data Generation** | Manual export | Built-in replicator for annotated datasets |
| **Domain Randomization** | Requires custom scripts | Native support with Python API |
| **GPU Acceleration** | Limited | Full GPU acceleration (physics + rendering) |
| **Real-World Transfer** | Moderate success | High success (photorealistic data) |

### Use Cases for Isaac Sim

1. **Algorithm Development**: Test navigation, manipulation, and control algorithms safely before hardware deployment
2. **Perception Training**: Generate millions of annotated images for training object detectors, semantic segmentation, depth estimation
3. **Demonstrations**: Create photorealistic videos and screenshots for papers, presentations, funding proposals
4. **Hardware-in-the-Loop**: Connect sim to real sensors/actuators for hybrid testing
5. **Reinforcement Learning**: Train RL policies in sim with physics randomization for robust real-world transfer

### Real-World Adoption

Isaac Sim powers humanoid development at leading robotics companies:

- **Agility Robotics (Digit)**: Uses Isaac Sim for warehouse navigation training
- **Boston Dynamics**: Leverages Isaac for perception pipeline development
- **Academic Research**: 100+ universities use Isaac Sim for humanoid research

---

## 2. Installation and Setup

Isaac Sim is distributed through **NVIDIA Omniverse**, a platform for 3D content creation and simulation.

### Step 1: Download NVIDIA Omniverse Launcher

1. Visit [https://www.nvidia.com/en-us/omniverse/download/](https://www.nvidia.com/en-us/omniverse/download/)
2. Sign in with your NVIDIA account (free registration)
3. Download the Omniverse Launcher for your OS (Windows or Linux)
4. Install the launcher (typical installation: `C:\Users\<YourName>\AppData\Local\ov\`)

### Step 2: Install Isaac Sim via Omniverse

1. Open Omniverse Launcher
2. Navigate to **Exchange** tab
3. Search for **"Isaac Sim"**
4. Click **Install** (downloads ~15GB, installs to `~/.local/share/ov/pkg/isaac_sim-*`)
5. Wait for installation to complete (10-30 minutes depending on internet speed)

### Step 3: Verify Installation

1. In Omniverse Launcher, go to **Library** tab
2. Find **Isaac Sim** in your installed apps
3. Click **Launch**
4. First launch takes 2-3 minutes (initializing shaders, loading assets)
5. You should see the Isaac Sim welcome screen with sample scenes

:::warning Hardware Requirements

**Minimum Requirements**:
- GPU: NVIDIA GTX 1060 (6GB VRAM) or better
- CPU: Intel i7 or AMD Ryzen 7 (4+ cores)
- RAM: 16GB (32GB recommended)
- Disk: 20GB free space (SSD recommended for fast asset loading)
- OS: Windows 10/11 or Ubuntu 20.04/22.04

**Recommended for Humanoid Simulation**:
- GPU: RTX 3060 or better (12GB+ VRAM for complex scenes)
- CPU: Intel i9 or AMD Ryzen 9 (8+ cores for physics)
- RAM: 32GB (64GB for large scenes with many robots)

**Cloud Alternatives** (if you don't have an NVIDIA GPU):
- **AWS EC2 g4dn instances**: Starting at $0.526/hour (Tesla T4 GPU, 16GB VRAM)
- **Google Cloud Platform**: N1 with NVIDIA T4 or V100 GPUs
- **Azure NV-series**: NC6 instances with Tesla K80

:::

### Step 4: Verify GPU Access

After launching Isaac Sim, verify GPU acceleration is working:

```python
# In Isaac Sim Python console (Window > Script Editor)
import carb
gpu_info = carb.settings.get_settings().get("/renderer/activeGpu")
print(f"Active GPU: {gpu_info}")
```

You should see your NVIDIA GPU model name. If it shows a CPU or integrated GPU, check your driver installation.

---

## 3. Importing Humanoid Models

Isaac Sim uses **USD (Universal Scene Description)** as its native format, but you can import URDF files (which you learned to create in Module 1 Chapter 3) and automatically convert them.

### Step-by-Step URDF Import

Let's import a humanoid robot URDF. If you don't have one, Isaac Sim provides sample robots in `/Isaac/Robots/` asset library.

**Method 1: GUI Import** (for exploring):

1. Open Isaac Sim
2. Go to **Isaac Utils > Workflows > URDF Importer**
3. Click **Select URDF File** and browse to your humanoid URDF (e.g., `humanoid.urdf`)
4. Configure import settings:
   - **Fix Base Link**: Uncheck (humanoids are mobile)
   - **Import Inertia Tensor**: Check (for accurate physics)
   - **Joint Drive Mode**: Position (for position-controlled joints)
5. Click **Import**
6. The robot appears in the scene, converted to USD

**Method 2: Python Script** (for automation):

```python
# Import URDF programmatically
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.importer.urdf import _urdf

# Path to your URDF file
urdf_path = "/path/to/your/humanoid.urdf"

# Import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomposition = False
import_config.import_inertia_tensor = True
import_config.fix_base = False  # Humanoid is mobile

# Perform import
_urdf.acquire_urdf_interface().parse_urdf(
    urdf_path,
    import_config,
    "/World/Humanoid"  # USD path where robot will appear
)

print("Humanoid robot imported successfully!")
```

### Verifying the Imported Model

After import, check that your humanoid robot is correctly configured:

1. **Scene Hierarchy**: Open **Stage** panel (Window > Stage). You should see `/World/Humanoid` with child links and joints
2. **Visual Inspection**: The robot should appear in the viewport with correct geometry
3. **Collision Shapes**: Toggle **Show > By Type > Colliders** to verify collision meshes
4. **Joint Articulation**: Select the robot, open **Property** panel, check that joints are listed with correct types (revolute, prismatic)

:::tip Common Import Issues

**Problem**: Robot appears but falls through ground
**Solution**: Ensure ground plane has collision enabled. Add **Physics > Ground Plane** from menu.

**Problem**: Robot texture/colors are missing
**Solution**: URDF materials don't directly convert to USD. You'll need to manually assign materials in Isaac Sim. Select mesh, go to **Material** property, and assign PBR materials.

**Problem**: Joints don't move
**Solution**: Verify joint drives are configured. Select joint in **Property** panel, check **Drive > Type** is set to Position or Velocity.

:::

---

## 4. Sensor Configuration

Isaac Sim provides a comprehensive suite of virtual sensors that simulate real robot sensors with high fidelity.

### Adding an RGB Camera

1. **Create Camera**:
   - Right-click on your robot's head link in the **Stage** panel
   - Select **Create > Camera**
   - Rename to `/World/Humanoid/head/rgb_camera`

2. **Position Camera**:
   - Select the camera
   - In **Property** panel, set **Transform**:
     - Position: `(0.1, 0, 0.05)` (10cm forward, 5cm up from head link)
     - Rotation: `(0, 0, 0)` (facing forward)

3. **Configure Camera Parameters**:
   - **Resolution**: 1280x720 (HD) or 640x480 (VGA)
   - **Horizontal FOV**: 90 degrees (wide-angle for humanoid perception)
   - **Vertical FOV**: Auto-calculated from aspect ratio
   - **Focal Length**: 24mm (standard for robotics)

4. **Enable Camera Output**:
   - With camera selected, go to **Isaac Sim > Render Products**
   - Add **RGB** render product
   - Set output topic (for ROS 2 integration): `/rgb_camera/image_raw`

### Adding a Depth Sensor

Depth sensors (like Intel RealSense or Azure Kinect) are critical for obstacle avoidance and VSLAM.

1. **Create Depth Camera**:
   - Right-click robot's head link
   - **Create > Camera** (name it `/World/Humanoid/head/depth_camera`)

2. **Position Next to RGB Camera**:
   - Position: `(0.1, -0.05, 0.05)` (5cm offset for stereo baseline)
   - Rotation: Same as RGB camera

3. **Add Depth Render Product**:
   - Select depth camera
   - **Isaac Sim > Render Products > Add Distance to Image Plane**
   - Set **Max Range**: 10.0 meters (typical for indoor navigation)
   - Output topic: `/depth_camera/depth`

### Adding LiDAR

LiDAR provides 360-degree 3D point clouds for mapping and obstacle detection.

1. **Create LiDAR Sensor**:
   - Right-click robot's torso link
   - **Isaac Sim > Create Sensor > Lidar > Rotating Lidar**

2. **Configure LiDAR Parameters**:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| **Number of Beams** | 16 or 32 | Vertical resolution (more beams = denser cloud) |
| **Rotation Rate** | 10 Hz | Scan frequency (10 full rotations per second) |
| **Max Range** | 20 meters | Detection distance |
| **Min Range** | 0.1 meters | Ignore points closer than this |
| **Horizontal Resolution** | 0.4 degrees | Angular step (900 points per rotation at 0.4°) |

3. **Position LiDAR**:
   - Mounted on torso at chest height
   - Position: `(0, 0, 0.8)` relative to base
   - Rotation: `(0, 0, 0)` (horizontal scan plane)

### Adding an IMU (Inertial Measurement Unit)

IMUs measure acceleration and angular velocity, essential for balance and state estimation.

1. **Create IMU Sensor**:
   - **Isaac Sim > Create Sensor > IMU**
   - Parent to robot's base link (center of mass)

2. **IMU Configuration**:
   - **Update Rate**: 100 Hz (typical for humanoid control)
   - **Noise Model**: Enable to simulate real sensor noise
     - Accelerometer noise: 0.01 m/s² std dev
     - Gyroscope noise: 0.001 rad/s std dev
   - Output topics:
     - `/imu/data` (combined accel + gyro)
     - `/imu/accel` (accelerometer only)
     - `/imu/gyro` (gyroscope only)

### Sensor Summary Table

| Sensor Type | Typical Location | Update Rate | Primary Use Case |
|-------------|------------------|-------------|------------------|
| **RGB Camera** | Head (forward-facing) | 30 Hz | Object detection, scene understanding |
| **Depth Camera** | Head (paired with RGB) | 30 Hz | Obstacle avoidance, VSLAM |
| **LiDAR** | Torso (chest height) | 10 Hz | 3D mapping, navigation |
| **IMU** | Base link (center of mass) | 100 Hz | Balance, odometry, state estimation |
| **Force Sensors** | Feet (contact points) | 100 Hz | Gait planning, balance control |

---

## 5. Running Simulations

Now that your humanoid robot is configured with sensors, let's run simulations in realistic environments.

### Creating Realistic Environments

Isaac Sim provides pre-built environment assets:

1. **Indoor Office Environment**:
   - **Assets > Environments > Simple Office**
   - Drag into scene at `/World/Office`
   - Includes desks, chairs, computers, walls

2. **Warehouse Environment**:
   - **Assets > Environments > Warehouse**
   - Large open space with shelving units, boxes, pallets
   - Ideal for navigation testing

3. **Outdoor Terrain**:
   - **Assets > Environments > Grid Room** (customize with outdoor textures)
   - Add terrain mesh with slopes, stairs, uneven ground
   - Skybox and directional lighting for outdoor ambiance

### Physics Settings

Configure physics for realistic humanoid simulation:

1. **Open Physics Settings**:
   - **Edit > Preferences > Physics**

2. **Key Parameters**:

| Setting | Value | Explanation |
|---------|-------|-------------|
| **Time Step** | 0.0083 seconds (120 Hz) | Physics update frequency (higher = more accurate) |
| **Gravity** | (0, 0, -9.81) m/s² | Earth gravity (negative Z is down) |
| **Solver Iterations** | 4 | Contact resolution quality (higher = more stable contacts) |
| **Friction Model** | Patch Friction | Realistic friction for foot-ground contact |
| **Ground Friction** | 0.8 | Coefficient for walking stability |

3. **Enable GPU Physics** (if supported):
   - **Physics > GPU Dynamics**: Check "Enabled"
   - Dramatically speeds up multi-robot simulations

### Playing the Simulation

1. **Press Play** (spacebar or toolbar button)
2. **Observe Robot Behavior**:
   - With no control input, humanoid should stand balanced (if properly configured)
   - Check joint limits are respected
   - Verify collision detection (robot doesn't penetrate ground)

3. **Adjust Simulation Speed**:
   - **Time Scale**: 1.0 = real-time, 0.5 = slow-motion, 2.0 = 2x speed
   - Useful for debugging fast motions or accelerating long tests

4. **Camera Controls**:
   - **Mouse**: Rotate view (left-click drag)
   - **WASD**: Move camera (FPS-style)
   - **Mouse Wheel**: Zoom in/out
   - **F**: Focus on selected object

### Visualizing Sensor Outputs

While simulation is running:

1. **View RGB Camera**:
   - Select `/World/Humanoid/head/rgb_camera`
   - **Window > Viewport** (creates a new viewport showing camera view)

2. **View Depth Data**:
   - Depth camera automatically publishes to ROS 2 topic (covered in Section 8)
   - Visualize in RViz2 (next module) or use Isaac Sim's **Synthetic Data Visualizer**

3. **View LiDAR Point Cloud**:
   - **Isaac Sim > Visualization > Point Cloud**
   - Select LiDAR sensor as source
   - Point cloud renders in 3D viewport in real-time

---

## 6. Synthetic Data Generation

One of Isaac Sim's most powerful features: automatically generating large-scale annotated datasets for training perception models.

### Setting Up Replicator for Dataset Export

**Replicator** is Isaac Sim's built-in tool for synthetic data generation.

```python
import omni.replicator.core as rep

# Define camera for data capture
camera = rep.create.camera(position=(2, 0, 1.5), look_at=(0, 0, 0.5))

# Define what to capture
render_product = rep.create.render_product(camera, resolution=(1280, 720))

# Annotators: What data to export
rep.AnnotatorRegistry.register_annotator("rgb")
rep.AnnotatorRegistry.register_annotator("bounding_box_2d_tight")
rep.AnnotatorRegistry.register_annotator("semantic_segmentation")
rep.AnnotatorRegistry.register_annotator("distance_to_camera")

# Writer: Where to save data
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="/path/to/dataset", rgb=True, bounding_box_2d_tight=True)

# Attach annotators to render product
rep.orchestrator.register_data_writer(writer, render_product)
```

### Dataset Format and Structure

After running Replicator, your dataset directory will look like:

```
/path/to/dataset/
├── rgb/
│   ├── 000000.png
│   ├── 000001.png
│   └── ... (10,000 images)
├── bounding_box_2d_tight/
│   ├── 000000.json  # {"objects": [{"label": "person", "bbox": [x, y, w, h]}, ...]}
│   ├── 000001.json
│   └── ...
├── semantic_segmentation/
│   ├── 000000.png  # Pixel-wise class labels (color-coded)
│   ├── 000001.png
│   └── ...
└── depth/
    ├── 000000.exr  # 16-bit depth maps (meters)
    ├── 000001.exr
    └── ...
```

### Annotation Types

| Annotation | File Format | Use Case |
|------------|-------------|----------|
| **RGB Images** | PNG (8-bit) | Visual appearance, texture, lighting |
| **Bounding Boxes** | JSON (x, y, width, height) | Object detection (YOLO, Faster R-CNN) |
| **Semantic Segmentation** | PNG (class IDs per pixel) | Pixel-level classification (DeepLab, U-Net) |
| **Instance Segmentation** | PNG (unique ID per object) | Instance-level classification (Mask R-CNN) |
| **Depth Maps** | EXR (32-bit float) | Monocular depth estimation, 3D reconstruction |
| **Normals** | EXR (XYZ vectors) | Surface orientation, 3D understanding |
| **Optical Flow** | EXR (motion vectors) | Video prediction, motion segmentation |

### Generating 1000 Images Example

```python
import omni.replicator.core as rep

# Register scene randomization (covered in Section 7)
with rep.trigger.on_frame(num_frames=1000):
    # Capture one frame per randomization
    rep.randomizer.scatter_3d(
        objects="/World/Objects/*",
        num_samples=10,
        surface_prims="/World/Ground"
    )

# Run simulation to generate dataset
rep.orchestrator.run()
```

This generates 1000 images with 10 randomly placed objects per frame—a complete training dataset in minutes!

### Dataset Validation

Before training perception models:

1. **Visual Inspection**: Open a few RGB images, verify they look photorealistic
2. **Annotation Check**: Load JSON bounding boxes, overlay on images, check accuracy
3. **Class Distribution**: Count objects per class, ensure balanced dataset (e.g., not 90% "person", 10% "robot")
4. **Diversity Check**: Verify lighting, object poses, backgrounds vary sufficiently

---

## 7. Domain Randomization

**Domain randomization** is the secret to successful sim-to-real transfer. By training on diverse randomized simulations, your model learns robust features that generalize to the real world.

### Why Domain Randomization Works

Real-world perception is hard because of **variations**:
- Lighting changes (morning sun vs evening shadows)
- Object appearances (red boxes vs blue boxes vs cardboard boxes)
- Camera noise (sensor degradation, motion blur)
- Background clutter (clean lab vs messy warehouse)

If you train on a single sim scene (fixed lighting, fixed textures), your model **overfits to simulation**. When deployed to real robots, it fails because real-world conditions don't match sim.

**Solution**: Randomize everything during training. Model learns invariant features (shape, geometry) instead of sim-specific artifacts.

### Lighting Randomization

```python
import omni.replicator.core as rep

def randomize_lighting():
    # Randomize directional light (sun)
    with rep.trigger.on_frame():
        # Intensity: 500-3000 lux (dawn to midday)
        rep.modify.attribute("/World/Sun", "intensity", rep.distribution.uniform(500, 3000))

        # Color temperature: 3000K-7000K (warm to cool)
        rep.modify.attribute("/World/Sun", "temperature", rep.distribution.uniform(3000, 7000))

        # Sun angle: -30° to +60° (sunrise to overhead)
        rep.modify.attribute("/World/Sun", "rotation",
            rep.distribution.uniform((-30, 0, -180), (60, 0, 180)))

randomize_lighting()
```

**Impact**: Model learns to detect objects in morning light, noon sun, overcast days, indoor lighting—robust to all conditions.

### Texture Randomization

```python
# Randomize object materials
def randomize_textures():
    objects = rep.get.prims(path_pattern="/World/Objects/*")

    with rep.trigger.on_frame():
        for obj in objects:
            # Random PBR material
            rep.randomizer.materials(
                materials=rep.get.prims(path_pattern="/World/Materials/*"),
                input_prims=obj,
                mode="random"  # Each object gets a random material each frame
            )

randomize_textures()
```

Materials include: wood, metal, plastic, fabric, concrete—hundreds of variations. Model learns "box shape" regardless of whether it's cardboard or aluminum.

### Object Placement Randomization

```python
# Randomize object positions and orientations
def randomize_object_placement():
    with rep.trigger.on_frame():
        rep.randomizer.scatter_3d(
            objects="/World/Objects/*",
            num_samples=rep.distribution.uniform(5, 15),  # 5-15 objects per scene
            surface_prims="/World/Ground",
            check_collisions=True  # Prevent objects from overlapping
        )

        # Random rotations
        rep.modify.pose(
            objects="/World/Objects/*",
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

randomize_object_placement()
```

Objects appear at random locations, random orientations—model learns 3D object understanding, not memorized positions.

### Full Randomization Pipeline

Combine all randomizations:

```python
import omni.replicator.core as rep

# Setup
camera = rep.create.camera(position=(3, 3, 2), look_at=(0, 0, 0.5))
render_product = rep.create.render_product(camera, resolution=(640, 480))

# Annotators
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="/data/randomized_dataset", rgb=True, bounding_box_2d_tight=True)
rep.orchestrator.register_data_writer(writer, render_product)

# Randomization triggers
with rep.trigger.on_frame(num_frames=10000):  # Generate 10,000 images
    # Lighting
    rep.modify.attribute("/World/Sun", "intensity", rep.distribution.uniform(500, 3000))
    rep.modify.attribute("/World/Sun", "rotation", rep.distribution.uniform((-30, 0, 0), (60, 0, 0)))

    # Textures
    rep.randomizer.materials(
        materials=rep.get.prims(path_pattern="/World/Materials/*"),
        input_prims=rep.get.prims(path_pattern="/World/Objects/*")
    )

    # Object placement
    rep.randomizer.scatter_3d(
        objects="/World/Objects/*",
        num_samples=rep.distribution.uniform(10, 20),
        surface_prims="/World/Ground"
    )

# Run
rep.orchestrator.run()
print("Generated 10,000 randomized training images!")
```

### Validation: Before vs After Randomization

| Metric | Fixed Scene (No Randomization) | Randomized Training |
|--------|-------------------------------|---------------------|
| **Sim Accuracy** | 98% (overfits to sim) | 85% (generalizes better) |
| **Real-World Accuracy** | 60% (sim-to-real gap) | 82% (transfers well!) |
| **Robustness** | Fails in new lighting | Works in varied conditions |

---

## 8. ROS 2 Integration

Isaac Sim integrates seamlessly with ROS 2, allowing you to control simulated robots and receive sensor data just like real hardware.

### Isaac Sim ROS 2 Bridge Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Isaac Sim (USD Scene)              │
│  ┌──────────────┐  ┌─────────────┐  ┌────────────┐ │
│  │ Humanoid     │  │   Sensors   │  │ Environment│ │
│  │   Robot      │  │ (Cam, LiDAR)│  │  (Office)  │ │
│  └──────┬───────┘  └──────┬──────┘  └────────────┘ │
│         │                  │                         │
│         └──────────┬───────┘                         │
│                    │                                 │
│         ┌──────────▼──────────┐                      │
│         │ Isaac ROS 2 Bridge  │                      │
│         │  (omni.isaac.ros2)  │                      │
│         └──────────┬──────────┘                      │
└────────────────────┼─────────────────────────────────┘
                     │
                     │ ROS 2 Topics/Services/Actions
                     ▼
        ┌────────────────────────────────┐
        │      ROS 2 Nodes (External)    │
        │  ┌──────────────────────────┐  │
        │  │  Navigation (Nav2)       │  │
        │  │  Perception (Isaac ROS)  │  │
        │  │  Control (Your Code)     │  │
        │  └──────────────────────────┘  │
        └────────────────────────────────┘
```

### Enabling ROS 2 Bridge

1. **Activate Isaac ROS 2 Extension**:
   - In Isaac Sim, go to **Window > Extensions**
   - Search for "ROS2"
   - Enable **omni.isaac.ros2_bridge**

2. **Verify ROS 2 Environment**:
   - Ensure ROS 2 is sourced in your terminal:
     ```bash
     source /opt/ros/humble/setup.bash  # or your ROS 2 distro
     ```

3. **Set ROS Domain ID** (if using multiple robots/computers):
   ```bash
   export ROS_DOMAIN_ID=42  # Use same ID for all Isaac Sim + ROS 2 nodes
   ```

### Publishing Sensor Data to ROS 2 Topics

**RGB Camera**:

```python
# In Isaac Sim Script Editor
from omni.isaac.ros2_bridge import ROS2Bridge

# Create ROS 2 camera publisher
camera_pub = ROS2Bridge.create_camera_publisher(
    camera_path="/World/Humanoid/head/rgb_camera",
    topic_name="/rgb_camera/image_raw",
    queue_size=10
)

print("RGB camera publishing to /rgb_camera/image_raw")
```

**Depth Camera**:

```python
depth_pub = ROS2Bridge.create_depth_publisher(
    camera_path="/World/Humanoid/head/depth_camera",
    topic_name="/depth_camera/depth",
    queue_size=10
)
```

**LiDAR**:

```python
lidar_pub = ROS2Bridge.create_lidar_publisher(
    lidar_path="/World/Humanoid/torso/lidar",
    topic_name="/scan",  # Standard ROS 2 LaserScan topic
    queue_size=10
)
```

### Subscribing to Control Commands

Control the humanoid robot from ROS 2 nodes:

```python
# Subscribe to velocity commands
from omni.isaac.ros2_bridge import ROS2Bridge

def velocity_callback(msg):
    # msg is geometry_msgs/Twist
    linear_vel = msg.linear.x
    angular_vel = msg.angular.z
    # Apply to robot base (covered in Module 1)
    print(f"Received velocity command: linear={linear_vel}, angular={angular_vel}")

cmd_vel_sub = ROS2Bridge.create_subscriber(
    topic_name="/cmd_vel",
    msg_type="geometry_msgs/Twist",
    callback=velocity_callback
)
```

### Launch File for Isaac Sim + ROS 2

Create a ROS 2 launch file to start Isaac Sim with automatic bridge setup:

```python
# launch/isaac_sim_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'isaac-sim.sh',  # Path to Isaac Sim executable
                '--/persistent/isaac/asset_root/default=/path/to/assets',
                '--/omni/replicator/autostart=true'
            ],
            output='screen'
        )
    ])
```

Launch with:

```bash
ros2 launch your_package isaac_sim_humanoid.launch.py
```

### Verifying ROS 2 Integration

Check that topics are publishing:

```bash
# List all ROS 2 topics
ros2 topic list

# Expected output:
# /rgb_camera/image_raw
# /depth_camera/depth
# /scan
# /imu/data
# /cmd_vel

# Check camera data rate
ros2 topic hz /rgb_camera/image_raw
# Expected: ~30 Hz

# Echo LiDAR data
ros2 topic echo /scan
```

---

## 9. Performance Tips

Isaac Sim is computationally intensive. Here's how to optimize for maximum performance:

### GPU Utilization Optimization

1. **Monitor GPU Usage**:
   ```bash
   nvidia-smi -l 1  # Update every 1 second
   ```
   - Look for **GPU Utilization**: Should be 80-100% during simulation
   - Look for **Memory Usage**: Should not exceed VRAM (causes slowdown)

2. **Reduce Scene Complexity** (if GPU maxed out):
   - Decrease number of objects in scene
   - Simplify mesh geometry (use collision proxies instead of visual meshes for physics)
   - Lower texture resolution (2K textures instead of 4K)

3. **Enable GPU Physics**:
   - **Edit > Preferences > Physics > GPU Dynamics**: Enable
   - Offloads physics computation from CPU to GPU (10-20x speedup for many objects)

### Cloud Deployment Options

If you don't have a local NVIDIA GPU or need scalable compute for large-scale data generation:

**AWS EC2 g4dn Instances**:

| Instance Type | GPU | VRAM | vCPUs | RAM | Cost/Hour (On-Demand) |
|---------------|-----|------|-------|-----|---------------------|
| g4dn.xlarge   | Tesla T4 | 16GB | 4 | 16GB | $0.526 |
| g4dn.2xlarge  | Tesla T4 | 16GB | 8 | 32GB | $0.752 |
| g4dn.12xlarge | 4x Tesla T4 | 64GB | 48 | 192GB | $3.912 |

**Setup Steps**:
1. Launch g4dn instance with Ubuntu 20.04 AMI
2. Install NVIDIA drivers: `sudo apt install nvidia-driver-535`
3. Install Docker: `sudo apt install docker.io nvidia-docker2`
4. Pull Isaac Sim container: `docker pull nvcr.io/nvidia/isaac-sim:2023.1.1`
5. Run Isaac Sim in headless mode (no GUI): `docker run --gpus all -v /data:/data isaac-sim ...`

### Headless Mode for Automation

For batch synthetic data generation (no need for GUI):

```bash
# Run Isaac Sim headless
isaac-sim.sh --headless --allow-root \
    --exec /path/to/your/replicator_script.py
```

This runs your Replicator script without loading the GUI, significantly faster for large-scale dataset generation (10,000+ images).

### Troubleshooting Common Performance Bottlenecks

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| **Low FPS (&lt;10)** | Scene too complex | Reduce mesh poly count, simplify colliders |
| **GPU at 100%, FPS still low** | VRAM exceeded (swapping to system RAM) | Lower texture resolution, reduce # of objects |
| **CPU bottleneck (GPU &lt;50%)** | Physics on CPU | Enable GPU Physics in settings |
| **Long load times** | Assets on HDD | Move Isaac Sim and scene assets to SSD |
| **Simulation unstable (robot jitters)** | Time step too large | Decrease physics time step (e.g., 120 Hz → 240 Hz) |

---

## Summary and Next Steps

Congratulations! You've learned how to use **NVIDIA Isaac Sim** for photorealistic humanoid robot simulation and synthetic data generation.

### What You Accomplished

- ✅ Installed Isaac Sim via NVIDIA Omniverse and verified GPU acceleration
- ✅ Imported a humanoid URDF model and converted it to USD for Isaac Sim
- ✅ Configured virtual sensors (RGB camera, depth, LiDAR, IMU) on the robot
- ✅ Ran simulations in realistic environments with accurate physics
- ✅ Generated annotated synthetic datasets (bounding boxes, segmentation masks, depth maps)
- ✅ Applied domain randomization (lighting, textures, object placement) for robust model training
- ✅ Integrated Isaac Sim with ROS 2 for sensor data publishing and robot control
- ✅ Optimized performance for GPU utilization and explored cloud deployment

### Key Takeaways

1. **Photorealistic Simulation Matters**: Isaac Sim's RTX rendering dramatically improves sim-to-real transfer compared to basic simulators
2. **Synthetic Data is Powerful**: Generate millions of annotated images in hours—impossible with real data collection
3. **Domain Randomization is Essential**: Randomize lighting, textures, and object placement to train robust perception models
4. **ROS 2 Integration is Seamless**: Isaac Sim publishes sensor data to ROS 2 topics just like real hardware

### What's Next?

In **Chapter 2: Isaac ROS**, you'll learn how to:

- Install **Isaac ROS** packages for GPU-accelerated perception
- Configure a **Visual SLAM (VSLAM) pipeline** using Isaac ROS for real-time localization and mapping
- Achieve **15+ FPS perception** (2-5x faster than standard ROS 2 packages)
- Visualize perception outputs (pose estimates, point clouds, feature tracks) in **RViz2**
- Tune Isaac ROS parameters for different environments (indoor vs outdoor)

Isaac Sim provides the simulation environment; Isaac ROS will give your humanoid robot **real-time perception** to understand and navigate the world.

---

**Chapter Navigation**:

- ← Previous: [Module 3 Overview](/docs/module-3-isaac/)
- → Next: [Chapter 2: Isaac ROS](/docs/module-3-isaac/isaac-ros)
- ↑ Back to Module 3: [Module 3 Index](/docs/module-3-isaac/)

---

*Chapter 1 of Module 3: The AI-Robot Brain (NVIDIA Isaac™)*
