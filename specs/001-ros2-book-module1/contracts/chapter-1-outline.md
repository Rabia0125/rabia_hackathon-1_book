# Chapter 1 Outline: ROS 2 Fundamentals

**File**: `docs/module-1-ros2/01-ros2-fundamentals.md`
**Word Count Target**: 5000-6000 words
**Reading Time**: ~25-30 minutes

## Frontmatter

```yaml
---
title: "ROS 2 Fundamentals: The Robotic Nervous System"
sidebar_position: 1
sidebar_label: "ROS 2 Fundamentals"
description: "Learn how ROS 2 acts as the nervous system of humanoid robots, enabling communication between sensors, actuators, and AI."
tags:
  - ros2
  - fundamentals
  - architecture
  - nervous-system
---
```

## Learning Objectives

After completing this chapter, you will be able to:
1. Explain what ROS 2 is and why it matters for humanoid robotics
2. Describe the nervous system analogy and map it to ROS 2 components
3. Differentiate between nodes, topics, services, and actions
4. Understand how DDS enables real-time communication
5. Identify which communication pattern to use for different scenarios

## Prerequisites

- Basic understanding of computer programming concepts
- Familiarity with Python (variables, functions, classes)
- Interest in robotics or AI (no prior ROS experience required)

## Content Sections

### 1. Introduction: Why ROS 2? (~500 words)
- The challenge of building robot software
- What problems ROS 2 solves
- Brief history: ROS 1 → ROS 2 evolution
- Real-world adoption (humanoid robots, autonomous vehicles)

**Callout**: "Coming from ROS 1?" sidebar with key differences

### 2. The Nervous System Metaphor (~800 words)
- Human nervous system overview (brain, nerves, signals)
- Mapping to robotics: sensors → processing → actuators
- Why this analogy helps understand ROS 2
- **Diagram**: Nervous system ↔ ROS 2 visual comparison

### 3. Nodes: The Neurons of Your Robot (~1000 words)
- What is a node?
- Node responsibilities (single purpose, modular)
- Node lifecycle (configured, active, inactive)
- Examples in humanoid robotics:
  - Vision node (camera processing)
  - Motor control node (joint commands)
  - Balance node (IMU processing)

**Diagram**: Multiple nodes communicating in a humanoid robot

### 4. Topics: Sensory Nerves Carrying Data (~1000 words)
- What is a topic?
- Publisher-subscriber pattern explained
- Message types (standard vs. custom)
- When to use topics:
  - Continuous sensor data (camera, lidar, IMU)
  - Streaming commands (velocity, joint positions)
- QoS basics (reliability, durability)

**Code Example**: Listing active topics with `ros2 topic list`

### 5. Services: Reflexes for Quick Responses (~800 words)
- What is a service?
- Request-response pattern explained
- When to use services:
  - Configuration changes
  - One-time queries
  - Calibration commands
- Service vs. topic comparison table

**Code Example**: Calling a service with `ros2 service call`

### 6. Actions: Motor Commands with Feedback (~800 words)
- What is an action?
- Goal-feedback-result pattern
- When to use actions:
  - Long-running tasks (walking, picking up objects)
  - Tasks that can be canceled
  - Tasks requiring progress updates
- Actions vs. services comparison

**Code Example**: Sending an action goal with `ros2 action send_goal`

### 7. DDS: The Backbone of Communication (~600 words)
- What is DDS (Data Distribution Service)?
- Why ROS 2 chose DDS over custom middleware
- Real-time guarantees and reliability
- Discovery mechanism (how nodes find each other)
- Brief mention of QoS profiles

**Diagram**: DDS layer in ROS 2 architecture stack

### 8. Putting It All Together (~400 words)
- Example: Humanoid robot standing up
  - IMU topic → Balance node → Motor service
- Decision flowchart: Topic vs. Service vs. Action

**Diagram**: Decision flowchart for communication patterns

## Summary / Key Takeaways

- ROS 2 is the "nervous system" that connects all parts of a robot
- **Nodes** = neurons (independent processing units)
- **Topics** = sensory nerves (continuous data streams)
- **Services** = reflexes (quick request-response)
- **Actions** = motor commands (long tasks with feedback)
- **DDS** provides the communication backbone with real-time guarantees

## Self-Assessment Questions

1. A camera is publishing images at 30 fps. Should this use a topic, service, or action?
2. You want to trigger a one-time calibration routine. Which pattern is best?
3. A humanoid robot needs to walk 10 meters with progress updates. Which pattern?
4. What happens if a subscriber joins a topic after a message was published?

## Requirement Coverage

| Requirement | Section | Status |
|-------------|---------|--------|
| FR-001: Nodes as neurons | Section 3 | Covered |
| FR-002: Topics as sensory nerves | Section 4 | Covered |
| FR-003: Services as reflexes | Section 5 | Covered |
| FR-004: Actions as long-running tasks | Section 6 | Covered |
| FR-005: DDS middleware | Section 7 | Covered |
| FR-006: Architecture diagram | Section 3, 7 | Covered |
| FR-020: Learning objectives | Header | Covered |
| FR-021: Summary section | Summary | Covered |
| FR-022: Nervous system metaphor | Section 2 | Covered |
