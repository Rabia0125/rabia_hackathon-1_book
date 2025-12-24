# Chapter 3 Outline: Humanoid Modeling with URDF

**File**: `docs/module-1-ros2/03-humanoid-urdf.md`
**Word Count Target**: 5500-6500 words
**Reading Time**: ~30 minutes

## Frontmatter

```yaml
---
title: "Humanoid Modeling with URDF"
sidebar_position: 3
sidebar_label: "Humanoid URDF"
description: "Learn to define humanoid robot structure using URDF, including links, joints, sensors, and best practices for bipedal robots."
tags:
  - ros2
  - urdf
  - humanoid
  - robot-modeling
  - joints
---
```

## Learning Objectives

After completing this chapter, you will be able to:
1. Explain the purpose and structure of URDF files
2. Define robot links with visual, collision, and inertial properties
3. Create joints connecting links with appropriate types and limits
4. Attach sensors (cameras, IMUs, force-torque) to URDF models
5. Apply humanoid-specific design patterns for bipedal robots
6. Read and interpret existing humanoid URDF files

## Prerequisites

- Chapter 1: ROS 2 Fundamentals (understanding of nodes and topics)
- Chapter 2: Python Agents (familiarity with ROS 2 message types)
- Basic understanding of 3D coordinate systems (x, y, z, roll, pitch, yaw)

## Content Sections

### 1. Introduction: Why Model Your Robot? (~400 words)
- The need for robot descriptions
- What URDF provides (kinematics, visualization, simulation)
- URDF vs. SDF vs. MJCF comparison
- Where URDF fits in the ROS 2 ecosystem

### 2. URDF Basics: XML Structure (~600 words)
- The `<robot>` root element
- Naming conventions
- File organization best practices
- Linking to mesh files

**Code Example**: Minimal URDF skeleton

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links and joints defined here -->
</robot>
```

### 3. Links: The Body Parts (~1000 words)
- What is a link?
- The `<visual>` element (appearance)
  - Primitive shapes: box, cylinder, sphere
  - Mesh files (.stl, .dae)
  - Material colors
- The `<collision>` element (physics boundaries)
  - Simplified collision geometries
  - Why collision differs from visual
- The `<inertial>` element (mass properties)
  - Mass, center of mass, inertia tensor
  - Why accurate inertials matter for humanoids

**Code Example**: Torso link with all properties

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0.2 0.2 0.8 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <origin xyz="0 0 0.1"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
  </inertial>
</link>
```

**Diagram**: Link anatomy (visual, collision, inertial components)

### 4. Joints: Connecting the Parts (~1200 words)
- What is a joint?
- Joint types explained:
  - `revolute`: Rotating with limits (elbows, knees)
  - `continuous`: Unlimited rotation (wheels)
  - `prismatic`: Linear sliding (telescoping arms)
  - `fixed`: No motion (sensor mounts)
- Parent-child relationships
- Joint origin and axis
- Limits: position, velocity, effort

**Code Example**: Shoulder joint (revolute)

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
</joint>
```

**Diagram**: Joint types visualization

### 5. Humanoid Kinematic Chains (~1000 words)
- Standard humanoid structure
- The importance of link hierarchy
- Degrees of freedom (DOF) per limb
- Common joint configurations:
  - Arm: shoulder (3 DOF) + elbow (1 DOF) + wrist (2 DOF)
  - Leg: hip (3 DOF) + knee (1 DOF) + ankle (2 DOF)
  - Head: neck (2 DOF)
- Base link considerations for bipedal robots

**Diagram**: Humanoid kinematic tree (15-link model)

### 6. Sensors in URDF (~800 words)
- Adding sensor links to the model
- Common sensor placements:
  - Head-mounted camera (vision)
  - Torso IMU (orientation)
  - Foot force-torque sensors (contact)
- Using `<gazebo>` tags for simulation sensors
- Sensor frame conventions

**Code Example**: Camera sensor attachment

```xml
<link name="head_camera">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
  </visual>
</link>

<joint name="head_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="head_camera"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo camera plugin (for simulation) -->
<gazebo reference="head_camera">
  <sensor type="camera" name="head_cam">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=head_camera/image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### 7. Humanoid-Specific Design Concerns (~800 words)
- Center of mass placement for balance
- Foot design for stable standing
- Arm reach and workspace
- Joint limit safety margins
- Symmetric vs. asymmetric designs
- Weight distribution strategies

**Callout**: "Tip: Keep the center of mass low and between the feet"

### 8. Complete Humanoid Example (~700 words)
- Introducing the 15-link humanoid model
- Walkthrough of the complete URDF
- How to visualize in RViz2
- How to spawn in Gazebo (teaser for future module)

**Code Example**: Simplified humanoid URDF (full file reference)

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.3 0.2 0.4"/></geometry>
      <material name="gray"><color rgba="0.5 0.5 0.5 1.0"/></material>
    </visual>
    <collision><geometry><box size="0.3 0.2 0.4"/></geometry></collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry><sphere radius="0.1"/></geometry>
      <material name="skin"><color rgba="0.9 0.8 0.7 1.0"/></material>
    </visual>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>

  <joint name="neck" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm Chain -->
  <link name="left_upper_arm">
    <visual><geometry><cylinder length="0.25" radius="0.03"/></geometry></visual>
    <collision><geometry><cylinder length="0.25" radius="0.03"/></geometry></collision>
    <inertial><mass value="1.5"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/></inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.18 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="30" velocity="2.0"/>
  </joint>

  <!-- Additional links and joints follow same pattern... -->
  <!-- See full URDF file in repository: static/urdf/simple_humanoid.urdf -->
</robot>
```

**Note**: Full 15-link URDF available in supplementary materials.

## Summary / Key Takeaways

- **URDF** defines robot structure in XML for ROS 2
- **Links** represent body parts with visual, collision, and inertial properties
- **Joints** connect links and define motion constraints
- Joint types: `revolute`, `continuous`, `prismatic`, `fixed`
- **Sensors** are attached as fixed-joint links with simulation plugins
- Humanoid design requires attention to **balance**, **reach**, and **symmetry**

## Self-Assessment Questions

1. What are the three property types every link should have?
2. Which joint type would you use for a humanoid knee?
3. Why might collision geometry differ from visual geometry?
4. Where would you place an IMU sensor on a humanoid?
5. What URDF element defines how far a joint can rotate?

## Requirement Coverage

| Requirement | Section | Status |
|-------------|---------|--------|
| FR-013: URDF XML structure | Section 2 | Covered |
| FR-014: Link properties | Section 3 | Covered |
| FR-015: Joint types | Section 4 | Covered |
| FR-016: Sensor attachment | Section 6 | Covered |
| FR-017: Humanoid concerns | Section 7 | Covered |
| FR-018: Humanoid URDF example | Section 8 | Covered |
| FR-020: Learning objectives | Header | Covered |
| FR-021: Summary section | Summary | Covered |
| FR-023: Syntax-highlighted code | All | Covered |
