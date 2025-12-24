# Chapter 2 Outline: Python Agents to Robot Control

**File**: `docs/module-1-ros2/02-python-ros-control.md`
**Word Count Target**: 6000-7000 words
**Reading Time**: ~30-35 minutes

## Frontmatter

```yaml
---
title: "Python Agents to Robot Control"
sidebar_position: 2
sidebar_label: "Python + ROS 2"
description: "Learn to connect your Python AI logic to ROS 2 using rclpy for publishing commands and subscribing to sensor data."
tags:
  - ros2
  - python
  - rclpy
  - publishers
  - subscribers
---
```

## Learning Objectives

After completing this chapter, you will be able to:
1. Create a ROS 2 node using the rclpy Python library
2. Publish velocity commands to control robot movement
3. Subscribe to sensor topics and process incoming data
4. Call ROS 2 services from Python code
5. Structure an AI agent that uses ROS 2 for perception and action

## Prerequisites

- Chapter 1: ROS 2 Fundamentals (nodes, topics, services concepts)
- Intermediate Python (classes, callbacks, type hints)
- ROS 2 Humble installed (or access to a ROS 2 environment)

## Content Sections

### 1. Introduction: Bridging AI and Robotics (~400 words)
- The gap between AI models and physical robots
- Where Python fits in the ROS 2 ecosystem
- rclpy: The official Python client library
- What we'll build: A simple perception-action agent

### 2. Setting Up Your Python Environment (~600 words)
- Creating a ROS 2 workspace
- Package structure for Python nodes
- Required imports and dependencies
- Verifying your setup

**Code Example**: Basic package structure and `setup.py`

### 3. Creating Your First Node (~800 words)
- The `Node` class explained
- Node initialization and naming
- The spin loop (keeping the node alive)
- Clean shutdown handling

**Code Example**: Minimal node that prints "Hello, ROS 2!"

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello, ROS 2!')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Publishing Commands: Controlling Movement (~1200 words)
- What is a publisher?
- Creating a publisher with `create_publisher()`
- Message types for motion (`geometry_msgs/Twist`)
- Timer-based publishing
- Publishing velocity commands to move a robot

**Code Example**: Velocity publisher (forward motion)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward at 0.5 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}')

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Diagram**: Publisher data flow (node → topic → subscribers)

### 5. Subscribing to Sensors: Receiving Data (~1200 words)
- What is a subscriber?
- Creating a subscriber with `create_subscription()`
- Callback functions for incoming messages
- Common sensor message types:
  - `sensor_msgs/Image` (cameras)
  - `sensor_msgs/Imu` (inertial measurement)
  - `sensor_msgs/LaserScan` (lidar)
- Processing sensor data in callbacks

**Code Example**: IMU subscriber

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg: Imu):
        orientation = msg.orientation
        self.get_logger().info(
            f'Orientation: x={orientation.x:.2f}, '
            f'y={orientation.y:.2f}, z={orientation.z:.2f}'
        )

def main():
    rclpy.init()
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6. Calling Services: Request-Response (~1000 words)
- When to use services (recap from Chapter 1)
- Creating a service client
- Synchronous vs. asynchronous calls
- Handling service responses
- Error handling for unavailable services

**Code Example**: Service client calling a trigger service

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceCaller(Node):
    def __init__(self):
        super().__init__('service_caller')
        self.client = self.create_client(Trigger, 'calibrate')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calibrate service...')

    def call_calibration(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Calibration successful!')
        else:
            self.get_logger().error(f'Calibration failed: {future.result().message}')

def main():
    rclpy.init()
    node = ServiceCaller()
    node.call_calibration()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7. Building an AI Agent (~1500 words)
- Agent architecture: Perception → Decision → Action
- Combining publishers and subscribers in one node
- Simple reactive behavior (obstacle avoidance)
- Adding decision logic between perception and action
- Best practices for AI integration

**Code Example**: Simple reactive agent

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ReactiveAgent(Node):
    def __init__(self):
        super().__init__('reactive_agent')

        # Perception: Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.perception_callback,
            10
        )

        # Action: Publish velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.min_distance = 0.5  # Stop if obstacle closer than 0.5m

    def perception_callback(self, msg: LaserScan):
        # Get minimum distance in front of robot
        front_ranges = msg.ranges[len(msg.ranges)//3:2*len(msg.ranges)//3]
        min_front = min(r for r in front_ranges if r > 0.1)

        # Decision: Stop or go?
        cmd = Twist()
        if min_front > self.min_distance:
            cmd.linear.x = 0.3  # Move forward
            self.get_logger().info(f'Clear ahead ({min_front:.2f}m), moving forward')
        else:
            cmd.linear.x = 0.0  # Stop
            self.get_logger().warn(f'Obstacle detected ({min_front:.2f}m), stopping')

        # Action: Publish command
        self.publisher.publish(cmd)

def main():
    rclpy.init()
    node = ReactiveAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Diagram**: Agent architecture (perception → decision → action loop)

### 8. Best Practices and Common Patterns (~500 words)
- Logging levels and when to use them
- Parameter handling for configurable behavior
- QoS settings for reliable communication
- Testing ROS 2 nodes

**Callout**: "Tip: Use launch files for complex setups" (future module teaser)

## Summary / Key Takeaways

- **rclpy** is the Python client library for ROS 2
- **Publishers** send commands (e.g., velocity) to topics
- **Subscribers** receive sensor data via callbacks
- **Service clients** make request-response calls
- **AI agents** combine perception (subscribers) and action (publishers)
- Always handle clean shutdown with `destroy_node()` and `shutdown()`

## Self-Assessment Questions

1. What method creates a publisher in rclpy?
2. How do you ensure your node keeps running to receive callbacks?
3. What's the difference between `spin()` and `spin_once()`?
4. When would you use `call_async()` vs. waiting for the service?
5. How would you add obstacle avoidance to the reactive agent?

## Requirement Coverage

| Requirement | Section | Status |
|-------------|---------|--------|
| FR-007: Create node with rclpy | Section 3 | Covered |
| FR-008: Publish velocity commands | Section 4 | Covered |
| FR-009: Subscribe to sensor data | Section 5 | Covered |
| FR-010: Call services from Python | Section 6 | Covered |
| FR-011: Structure AI agent | Section 7 | Covered |
| FR-012: Complete code examples | All | Covered |
| FR-020: Learning objectives | Header | Covered |
| FR-021: Summary section | Summary | Covered |
| FR-023: Syntax-highlighted code | All | Covered |
