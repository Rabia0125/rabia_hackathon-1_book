---
sidebar_position: 3
slug: nav2-humanoids
title: "Nav2 for Humanoids: Bipedal Navigation"
sidebar_label: "Nav2 for Humanoids"
description: "Learn how to use Nav2 for path planning and autonomous navigation adapted for bipedal humanoid robots with unique constraints."
tags:
  - nav2
  - navigation
  - path-planning
  - bipedal-robots
  - obstacle-avoidance
---

# Nav2 for Humanoids: Bipedal Navigation

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain what Nav2 is, why it's the standard ROS 2 navigation stack, and what makes bipedal navigation unique compared to wheeled robots
2. Install and configure Nav2 packages for a humanoid robot
3. Configure Nav2 costmaps (global and local) using perception data from Isaac ROS (Chapter 2)
4. Understand Nav2 behavior tree architecture and key plugins (planners, controllers, recovery behaviors)
5. Configure bipedal-specific constraints (step height limits, foothold stability, balance considerations)
6. Send navigation goals to Nav2 using action clients and monitor navigation progress
7. Understand path planning algorithms used in Nav2 (A*, Dijkstra, Smac Planner) and when to use each
8. Implement local trajectory planning and obstacle avoidance as the robot moves toward goals
9. Handle dynamic obstacles through costmap updates and replanning
10. Configure recovery behaviors (rotate, back up, clear costmap) for stuck situations
11. Execute full navigation workflow in Isaac Sim: set goal → plan → execute → avoid obstacles → reach goal
12. Apply safety considerations when transitioning from simulation to physical humanoid hardware

## Prerequisites

:::info Before You Begin

- **Chapter 1 (Isaac Sim)**: Understanding of Isaac Sim environment setup and simulation → [Chapter 1: Isaac Sim](/docs/module-3-isaac/isaac-sim)
- **Chapter 2 (Isaac ROS perception)**: Experience with Isaac ROS VSLAM, odometry, and point cloud generation → [Chapter 2: Isaac ROS](/docs/module-3-isaac/isaac-ros)
- **Module 1 Chapter 2 (ROS 2 actions)**: Understanding of ROS 2 action interfaces for goal-based control → [Module 1 Chapter 2](/docs/module-1-ros2/02-python-ros-control)
- **Basic Navigation Concepts**: Familiarity with maps, localization, and path planning from Module 2

:::

---

## 1. Introduction: Bipedal Navigation Challenges

You've built the perception foundation with Isaac Sim and Isaac ROS (Chapters 1-2). Now let's enable **autonomous navigation**—the ability for your humanoid robot to plan paths and reach goals while avoiding obstacles.

### Why Navigation is Harder for Humanoid Robots

Wheeled robots (like delivery robots or warehouse AGVs) can:
- Turn in place effortlessly (differential drive)
- Ignore small obstacles (roll over cables, thresholds)
- Maintain stability easily (low center of mass, wide wheelbase)

Humanoid robots face unique challenges:

| Challenge | Wheeled Robot | Bipedal Humanoid |
|-----------|---------------|------------------|
| **Turning** | Rotate in place (0 turning radius) | Must walk in arc or take steps (turning radius > 0.5m) |
| **Obstacles** | Roll over small bumps (&lt;5cm) | Must step over (max step height ~15-20cm) |
| **Terrain** | Flat surfaces only | Stairs, slopes, uneven ground (if capable) |
| **Stability** | Always stable (static support) | Dynamically balanced (falls if miscalculated) |
| **Foothold** | Don't care about ground contact | Must place feet on stable surfaces (no ice, no gaps) |
| **Speed Changes** | Instant acceleration/deceleration | Must maintain momentum for balance |

**Nav2 wasn't designed for humanoids**—it was built for wheeled robots. But with proper configuration and custom plugins, Nav2 can work for bipedal navigation.

### What Makes Bipedal Navigation Complex?

**1. Step Height Constraints**:
- Humanoid can only step over obstacles up to ~20cm (knee bend limit)
- Must detect and route around taller obstacles (boxes, furniture)

**2. Foothold Planning**:
- Each foot placement must be on stable, flat surface
- Can't step on ice, loose gravel, or narrow ledges
- Requires terrain classification (beyond standard Nav2)

**3. Balance Requirements**:
- Center of mass must stay above support polygon (feet area)
- Sharp turns risk tipping over
- Acceleration/deceleration limits for stability

**4. Kinematic Constraints**:
- Cannot move sideways (holonomic) like omni-wheel robots
- Forward/backward + rotation only (nonholonomic)
- Minimum turning radius (can't spin in place)

### Nav2 + Custom Plugins = Humanoid Navigation

Nav2 provides the framework; you add humanoid-specific logic:

```
┌──────────────────────────────────────────────────────────────┐
│                       Nav2 Stack (Core)                       │
│  ┌────────────────┬────────────────┬─────────────────────┐   │
│  │  Path Planners │   Controllers  │ Recovery Behaviors  │   │
│  │  (Global path) │ (Local control)│ (Stuck handling)    │   │
│  └────────────────┴────────────────┴─────────────────────┘   │
│                            ▲                                  │
│                            │                                  │
│           ┌────────────────┴────────────────┐                │
│           │   Costmaps (Obstacle Info)      │                │
│           │   - Global (static map)         │                │
│           │   - Local (dynamic obstacles)   │                │
│           └────────────────┬────────────────┘                │
│                            ▲                                  │
└────────────────────────────┼──────────────────────────────────┘
                             │
                ┌────────────┴────────────┐
                │   Perception (Isaac ROS)│
                │   - Odometry            │
                │   - Point Clouds        │
                │   - Obstacle Detection  │
                └─────────────────────────┘
```

---

## 2. Installation and Setup

Nav2 is part of the standard ROS 2 distribution.

### Install Nav2 Packages

```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-nav2-bt-navigator \
                 ros-humble-nav2-planner \
                 ros-humble-nav2-controller \
                 ros-humble-nav2-costmap-2d \
                 ros-humble-nav2-recoveries \
                 ros-humble-nav2-lifecycle-manager

# Visualization tools
sudo apt install ros-humble-rviz2 \
                 ros-humble-nav2-rviz-plugins

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Verify Nav2 Installation

```bash
# Check Nav2 packages
ros2 pkg list | grep nav2

# Expected output (partial):
# nav2_bt_navigator
# nav2_planner
# nav2_controller
# nav2_costmap_2d
# ...

# Check Nav2 launch files
ros2 launch nav2_bringup --show-args

# Expected: Launch argument descriptions
```

### Nav2 Architecture Overview

Nav2 is organized into **nodes** (processes) and **plugins** (loadable modules):

```
Nav2 Nodes:
├── bt_navigator        (Behavior Tree executor, coordinates everything)
├── planner_server      (Global path planning: map → path)
├── controller_server   (Local trajectory control: path → cmd_vel)
├── recoveries_server   (Stuck recovery: rotate, backup, clear costmap)
├── lifecycle_manager   (Manages node lifecycle: configure, activate)
└── waypoint_follower   (Multi-waypoint navigation)

Nav2 Plugins (loadable):
├── Planners: NavFn, SmacPlanner, ThetaStar, ...
├── Controllers: DWB, TEB, MPPI, RPP, ...
└── Recovery Behaviors: Spin, BackUp, Wait, ClearCostmap, ...
```

**Key Insight**: You don't modify Nav2 code—you configure which plugins to use and tune their parameters for humanoid robots.

---

## 3. Costmap Configuration

**Costmaps** are 2D grids representing obstacle information. Nav2 uses costmaps for path planning and collision avoidance.

### Global vs Local Costmaps

| Costmap Type | Purpose | Size | Update Rate | Data Source |
|--------------|---------|------|-------------|-------------|
| **Global** | Long-range path planning | Large (entire map, e.g., 100m × 100m) | Slow (1 Hz) | Static map + SLAM map |
| **Local** | Short-range obstacle avoidance | Small (5m × 5m around robot) | Fast (10 Hz) | Real-time sensors (LiDAR, depth camera) |

### Costmap Layers

Costmaps are built from multiple **layers** (each layer adds obstacle info):

1. **Static Layer**: Pre-built map (from SLAM or CAD)
2. **Obstacle Layer**: Dynamic obstacles from sensors (point clouds, laser scans)
3. **Inflation Layer**: Expands obstacles (safety margin for robot footprint)
4. **Voxel Layer**: 3D obstacle representation (for overhead obstacles)

### Configuring Global Costmap

Create `config/nav2_params.yaml`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # Coordinate frames
      global_frame: map
      robot_base_frame: base_link

      # Costmap size and resolution
      width: 100  # meters
      height: 100
      resolution: 0.05  # 5cm per cell (finer = more accurate, slower)

      # Update rates
      update_frequency: 1.0  # Hz (slow for global)
      publish_frequency: 1.0

      # Layers (order matters!)
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map  # From Isaac ROS VSLAM (Chapter 2)
        subscribe_to_updates: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: pointcloud
        pointcloud:
          topic: /visual_slam/vis/landmarks  # Isaac ROS point cloud
          sensor_frame: camera_optical_frame
          data_type: "PointCloud2"
          marking: true  # Mark obstacles
          clearing: true  # Clear obstacles when not seen
          max_obstacle_height: 2.0  # meters (ignore ceiling)
          min_obstacle_height: 0.1  # meters (ignore floor noise)

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55  # meters (robot radius + safety margin)
        cost_scaling_factor: 3.0  # Higher = steeper cost gradient
```

### Configuring Local Costmap

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link

      # Smaller, high-resolution costmap
      width: 5  # meters (5m × 5m around robot)
      height: 5
      resolution: 0.02  # 2cm per cell (finer for local precision)

      # Faster updates for real-time obstacle avoidance
      update_frequency: 10.0  # Hz
      publish_frequency: 10.0

      # Rolling window (follows robot)
      rolling_window: true

      plugins: ["obstacle_layer", "inflation_layer"]  # No static layer (uses global for that)

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: pointcloud lidar
        pointcloud:
          topic: /visual_slam/vis/landmarks
          sensor_frame: camera_optical_frame
          data_type: "PointCloud2"
          marking: true
          clearing: true
          max_obstacle_height: 2.0
          min_obstacle_height: 0.15  # Higher than global (ignore low curbs robot can step over)
        lidar:
          topic: /scan  # From Isaac Sim LiDAR (Chapter 1)
          sensor_frame: lidar_frame
          data_type: "LaserScan"
          marking: true
          clearing: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.45  # Slightly smaller for local (allows tighter maneuvering)
        cost_scaling_factor: 5.0
```

### Costmap Cell Values

Each costmap cell has a value 0-255:

| Value | Meaning | Color (RViz) |
|-------|---------|--------------|
| 0 | Free space (safe to traverse) | White |
| 1-127 | Low cost (near obstacles) | Light gray → dark gray |
| 128-252 | High cost (very close to obstacles) | Dark gray |
| 253 | Inscribed (robot touches obstacle if centered here) | Magenta |
| 254 | Obstacle (occupied by obstacle) | Black |
| 255 | Unknown (not observed yet) | Blue |

---

## 4. Behavior Trees and Plugins

Nav2 uses **Behavior Trees (BTs)** to coordinate navigation logic. Think of BTs as flowcharts that decide "what to do next."

### Nav2 Behavior Tree Structure

```xml
<!-- nav2_bt.xml -->
<BehaviorTree>
  <NavigateToPose goal="{goal}" result="{result}">
    <!-- Main navigation sequence -->
    <Sequence>
      <!-- 1. Plan global path -->
      <ComputePathToPose goal="{goal}" path="{path}"/>

      <!-- 2. Follow path with local control -->
      <FollowPath path="{path}"/>

      <!-- 3. If stuck, try recovery -->
      <FallbackNode>
        <Sequence>
          <!-- Recovery 1: Rotate to find space -->
          <Spin spin_dist="1.57"/>  <!-- 90 degrees -->

          <!-- Recovery 2: Back up -->
          <BackUp backup_dist="0.5" backup_speed="0.1"/>

          <!-- Recovery 3: Clear costmap and retry -->
          <ClearCostmapExceptRegion/>
        </Sequence>
      </FallbackNode>
    </Sequence>
  </NavigateToPose>
</BehaviorTree>
```

**Key BT Nodes**:
- **Sequence**: Execute children in order; fail if any child fails
- **Fallback**: Execute children in order; succeed if any child succeeds (used for recovery)
- **ComputePathToPose**: Call planner_server to plan global path
- **FollowPath**: Call controller_server to execute path
- **Recovery Actions**: Spin, BackUp, Wait, ClearCostmap

### Key Plugins

**1. Planner Plugins** (global path planning):

| Plugin | Algorithm | Use Case |
|--------|-----------|----------|
| **NavFn** | Dijkstra (grid-based) | Simple, guaranteed complete path |
| **SmacPlanner2D** | Hybrid A* (kinematic-aware) | **Best for humanoids** (respects turning radius) |
| **ThetaStar** | Any-angle A* | Smooth paths (no grid alignment) |

**2. Controller Plugins** (local trajectory execution):

| Plugin | Method | Use Case |
|--------|--------|----------|
| **DWB** | Dynamic Window Approach | **Best for humanoids** (velocity-based, fast) |
| **TEB** | Timed Elastic Band | Smooth trajectories (slower computation) |
| **RPP** | Regulated Pure Pursuit | Simple car-like following |

**3. Recovery Plugins**:
- **Spin**: Rotate in place to find clear path
- **BackUp**: Reverse out of tight space
- **Wait**: Pause for dynamic obstacles to move
- **ClearCostmap**: Reset costmap if spurious obstacles

---

## 5. Bipedal-Specific Constraints

Standard Nav2 assumes wheeled robots. Here's how to configure for humanoid constraints:

### Robot Footprint

Define humanoid's 2D footprint (for collision checking):

```yaml
# In nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Footprint: polygon vertices (x, y) in base_link frame
      footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
      # Rectangle: 0.6m (front-back) × 0.4m (side-to-side)
      # Represents humanoid's stance width + safety margin
```

**For irregular humanoid shapes**:
```yaml
# More accurate humanoid outline
footprint: "[[0.35, 0.15], [0.3, 0.2], [0.2, 0.25], [-0.2, 0.25], [-0.3, 0.2], [-0.35, 0.15], [-0.35, -0.15], [-0.3, -0.2], [-0.2, -0.25], [0.2, -0.25], [0.3, -0.2], [0.35, -0.15]]"
# Octagon approximating humanoid shoulders + hips
```

### Step Height Limits

Nav2 doesn't natively understand step height. Workaround: **mark tall obstacles as impassable** in costmap.

```yaml
obstacle_layer:
  ros__parameters:
    min_obstacle_height: 0.05  # Ignore floor noise
    max_obstacle_height: 0.20  # ← CRITICAL: Humanoid max step height
    # Obstacles taller than 20cm are marked as impassable
    # Obstacles 5-20cm are considered traversable (robot can step over)
```

**For advanced step planning** (future work):
- Use 3D costmap (voxel layer)
- Implement custom controller that checks step height before foot placement
- Integrate with locomotion planner (whole-body motion planning)

### Kinematic Constraints (Turning Radius)

Humanoids can't spin in place like differential-drive robots. Configure minimum turning radius:

```yaml
controller_server:
  ros__parameters:
    DWB:
      plugin: "dwb_core::DWBLocalPlanner"
      # Velocity limits
      max_vel_x: 0.5  # m/s (humanoid walk speed)
      min_vel_x: 0.0  # Can't go backward easily
      max_vel_y: 0.0  # Non-holonomic (no sideways)
      max_vel_theta: 0.5  # rad/s (turning rate)
      min_vel_theta: -0.5

      # Acceleration limits (for balance)
      acc_lim_x: 0.3  # m/s² (slow accel to maintain balance)
      acc_lim_theta: 0.5  # rad/s²

      # Minimum turning radius (arc constraint)
      min_in_place_vel_theta: 0.4  # Avoid in-place rotation (not stable for humanoids)
```

### Balance Considerations

Nav2 doesn't model balance, but you can constrain commands for stability:

```yaml
DWB:
  # Trajectory scoring (prefer smooth, stable paths)
  critics: ["PathAlign", "GoalAlign", "PathDist", "GoalDist", "Oscillation", "Twirling"]
  Oscillation:
    scale: 1.0  # Penalize back-and-forth motion (unstable for humanoids)
  Twirling:
    scale: 1.0  # Penalize spinning (balance risk)
```

**For true balance-aware navigation** (beyond Nav2):
- Integrate with whole-body controller (e.g., Model Predictive Control)
- Use ZMP (Zero Moment Point) constraint in trajectory optimization
- Requires custom Nav2 controller plugin (C++)

---

## 6. Sending Navigation Goals

Use Nav2's **action interface** to command the robot to navigate to a goal pose.

### NavigateToPose Action (Python)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def navigate_to_goal(self, x, y, yaw):
        # Wait for action server
        self.nav_to_pose_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation (yaw to quaternion)
        import math
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        # Send goal
        self.get_logger().info(f'Navigating to: x={x}, y={y}, yaw={yaw}')
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f}m')

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached!')
        else:
            self.get_logger().error('Navigation failed!')

def main():
    rclpy.init()
    navigator = HumanoidNavigator()

    # Navigate to goal (5m forward, 3m right, facing 90° left)
    navigator.navigate_to_goal(x=5.0, y=3.0, yaw=1.57)

    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
```

### Monitoring Navigation Progress

```python
# In feedback_callback:
feedback = feedback_msg.feedback
distance = feedback.distance_remaining  # meters to goal
elapsed = feedback.navigation_time  # seconds since start
self.get_logger().info(f'Progress: {distance:.1f}m remaining, {elapsed.sec}s elapsed')
```

### Canceling Navigation

```python
# Cancel current goal
goal_handle.cancel_goal_async()
self.get_logger().info('Navigation canceled')
```

---

## 7. Path Planning Algorithms

Nav2 supports multiple global planners. Choose based on your humanoid's capabilities:

### A* Planner (NavFn)

**Algorithm**: A* on grid (8-connected: up, down, left, right, diagonals)

**Pros**:
- Fast (milliseconds for 100m × 100m map)
- Complete (always finds path if one exists)
- Simple to tune

**Cons**:
- Grid-aligned paths (zigzag, not smooth)
- No kinematic constraints (assumes holonomic robot)

**Configuration**:
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # meters (goal tolerance)
      use_astar: true  # vs Dijkstra (A* is faster)
```

**When to use**: Simple indoor environments, wheeled-like humanoids (can turn in place)

### Smac Planner (Hybrid A*)

**Algorithm**: Hybrid A* with kinematic constraints (respects turning radius)

**Pros**:
- **Kinematic-aware**: Plans paths humanoid can actually follow
- Smooth curves (no zigzag)
- Reverse motion support (if humanoid can walk backward)

**Cons**:
- Slower than NavFn (10-100ms)
- More parameters to tune

**Configuration**:
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["SmacPlanner2D"]
    SmacPlanner2D:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.25
      downsample_costmap: false  # Use full resolution
      allow_unknown: true  # Plan through unknown space (explore)

      # Humanoid kinematic constraints
      minimum_turning_radius: 0.6  # meters (humanoid can't spin in place)
      max_iterations: 1000000  # Search limit
      smooth_path: true  # Post-process for smoothness

      # Search heuristics
      analytic_expansion_ratio: 3.5  # When to try direct path
```

**When to use**: **Best for humanoids** (respects turning radius, smooth paths)

### Theta* Planner

**Algorithm**: Any-angle A* (paths not constrained to grid)

**Pros**:
- Smooth paths (straight lines when possible)
- Shorter paths than grid-based A*

**Cons**:
- No kinematic constraints (like NavFn)
- Requires line-of-sight checks (slower)

**When to use**: Open environments with few obstacles, wheeled-like humanoids

### Comparison Table

| Planner | Turning Radius | Path Smoothness | Speed | Humanoid-Suitable? |
|---------|----------------|-----------------|-------|-------------------|
| **NavFn** | ❌ No | ⭐ Low (grid-aligned) | ⚡ Fast (1-5ms) | ⚠️ OK for wheeled-like |
| **SmacPlanner2D** | ✅ Yes | ⭐⭐⭐ High (curves) | ⚡⚡ Medium (10-50ms) | ✅ **Best for humanoids** |
| **ThetaStar** | ❌ No | ⭐⭐ Medium (any-angle) | ⚡⚡ Medium (20-100ms) | ⚠️ OK for wheeled-like |

---

## 8. Local Trajectory Planning

Once global planner generates a path, the **controller** executes it by sending velocity commands (`cmd_vel`).

### DWB Controller (Dynamic Window Approach)

**How it works**:
1. Sample velocity trajectories (forward vel × angular vel grid)
2. Simulate each trajectory for 1-2 seconds
3. Score trajectories (prefer: close to path, avoid obstacles, reach goal)
4. Pick best trajectory → send velocity command
5. Repeat at 10 Hz

**Configuration**:
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Velocity search space
      vx_samples: 20  # Forward velocity samples
      vy_samples: 0   # No sideways (non-holonomic)
      vtheta_samples: 20  # Angular velocity samples

      # Simulation
      sim_time: 1.5  # seconds (simulate trajectory forward)
      sim_granularity: 0.025  # seconds (time step)

      # Trajectory critics (scoring functions)
      critics: ["RotateToGoal", "Oscillation", "PathAlign", "GoalAlign", "PathDist", "GoalDist"]

      # Weights (tune for humanoid behavior)
      PathAlign.scale: 32.0      # Stay close to path
      GoalAlign.scale: 24.0      # Face goal when near
      PathDist.scale: 32.0       # Make progress along path
      GoalDist.scale: 24.0       # Approach goal
      Oscillation.scale: 1.0     # Penalize back-and-forth (balance!)
      RotateToGoal.scale: 1.0    # Rotate to face goal at end
```

### TEB Controller (Timed Elastic Band)

**How it works**:
- Optimizes entire trajectory (not just next velocity)
- Considers dynamic constraints (acceleration limits)
- Smoother than DWB, but slower computation

**When to use**: Humanoids with whole-body control (can handle optimized trajectories)

**Configuration**:
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "teb_local_planner::TebLocalPlannerROS"

      # Trajectory optimization
      teb_autosize: true
      dt_ref: 0.3  # Desired time resolution (seconds)
      dt_hysteresis: 0.1

      # Humanoid constraints
      max_vel_x: 0.5
      max_vel_x_backwards: 0.0  # Humanoid can't walk backward easily
      max_vel_theta: 0.5
      acc_lim_x: 0.3  # Limit acceleration for balance
      acc_lim_theta: 0.5

      # Optimization weights
      weight_kinematics_forward_drive: 1.0  # Prefer forward motion
      weight_optimaltime: 1.0  # Minimize time
```

---

## 9. Dynamic Obstacle Avoidance

Nav2 handles moving obstacles by updating the local costmap in real-time.

### How It Works

```
┌─────────────────────────────────────────────────────────┐
│  1. Sensors detect moving obstacle (person walking)    │
│     → LiDAR/camera point cloud updated at 10-30 Hz     │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│  2. Obstacle layer updates local costmap                │
│     → New obstacle cells marked (value = 254)           │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│  3. Controller re-scores trajectories                   │
│     → Trajectories through obstacle get high cost       │
│     → Controller picks trajectory that avoids obstacle  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│  4. Robot adjusts path in real-time                     │
│     → Slows down, curves around person, continues       │
└─────────────────────────────────────────────────────────┘
```

### Configuring Dynamic Avoidance

Key parameters in `obstacle_layer`:

```yaml
obstacle_layer:
  ros__parameters:
    # Fast updates for dynamic obstacles
    observation_sources: lidar pointcloud

    lidar:
      topic: /scan
      data_type: LaserScan
      expected_update_rate: 0.1  # 10 Hz (fast)
      marking: true  # Mark obstacles immediately
      clearing: true  # Clear obstacles when not seen (for moving obstacles)
      raytrace_range: 10.0  # meters (clear cells along sensor rays)
      obstacle_range: 8.0   # meters (mark obstacles within this range)

    pointcloud:
      topic: /visual_slam/vis/landmarks
      data_type: PointCloud2
      expected_update_rate: 0.2  # 5 Hz
      marking: true
      clearing: true
      max_obstacle_height: 2.0  # Detect people (1.8m tall)
      min_obstacle_height: 0.5  # Ignore low clutter
```

### Replanning Triggers

Nav2 automatically replans the global path if:

1. **Local path blocked**: Controller can't find collision-free trajectory
2. **Path significantly deviated**: Robot drifted >0.5m from planned path
3. **New obstacle in path**: Costmap update detects obstacle along path

**Configuration**:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_robot_pose_search_dist: 1.0  # meters (how far off path before replan)
      prune_plan: true  # Remove passed waypoints (don't backtrack)

planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Hz (replan rate if needed)
```

### Example: Navigating Through Crowd

**Scenario**: Humanoid navigating office, person walks across path.

**Timeline**:
- **T=0s**: Global path planned (A to B, straight line)
- **T=5s**: Local controller detects person at 3m ahead (LiDAR)
- **T=5.1s**: Local costmap updates (person marked as obstacle)
- **T=5.2s**: Controller re-scores trajectories, picks curve around person
- **T=6-8s**: Robot curves around person (slows to 0.3 m/s for safety)
- **T=8s**: Person clears path, local costmap clears obstacle
- **T=8.5s**: Controller returns to original path, speeds up to 0.5 m/s
- **T=12s**: Robot reaches goal

**No replanning needed** (local controller handled it). Global replan only if person stops in path for >2 seconds.

---

## 10. Recovery Behaviors

When Nav2 gets stuck (no collision-free trajectory), recovery behaviors activate.

### Recovery Sequence

```yaml
bt_navigator:
  ros__parameters:
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

    # Recovery behaviors (executed in order when stuck)
    recovery_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_recoveries/Spin"
      simulate_ahead_time: 2.0  # seconds (check if spin is safe)

    backup:
      plugin: "nav2_recoveries/BackUp"
      simulate_ahead_time: 2.0

    wait:
      plugin: "nav2_recoveries/Wait"
      wait_duration: 5  # seconds (wait for dynamic obstacle to move)
```

### Recovery 1: Spin (Rotate to Find Clear Path)

**When triggered**: Controller can't find forward trajectory (surrounded by obstacles)

**Action**: Rotate 360° slowly, scanning for clear path

**Humanoid considerations**:
- Spinning risks balance loss (high angular velocity)
- Limit rotation speed: `max_rotational_vel: 0.3 rad/s`
- If unstable, skip spin, go straight to backup

**Configuration**:
```yaml
spin:
  plugin: "nav2_recoveries/Spin"
  simulate_ahead_time: 2.0
  max_rotational_vel: 0.3  # rad/s (slow for humanoid stability)
  min_rotational_vel: 0.1
  rotational_acc_lim: 0.2  # rad/s² (gradual acceleration)
```

### Recovery 2: Back Up (Reverse Out of Tight Space)

**When triggered**: Spin didn't work, still stuck

**Action**: Move backward 0.5m, retry forward motion

**Humanoid considerations**:
- Backward walking is hard for humanoids (limited ankle dorsiflexion)
- Use very slow backup speed: `backup_speed: 0.1 m/s`
- Short backup distance: `backup_dist: 0.3m` (just enough to create space)

**Configuration**:
```yaml
backup:
  plugin: "nav2_recoveries/BackUp"
  simulate_ahead_time: 2.0
  backup_dist: 0.3  # meters (short backup for humanoid)
  backup_speed: 0.1  # m/s (very slow, maintain balance)
```

### Recovery 3: Clear Costmap (Reset Spurious Obstacles)

**When triggered**: Spin and backup both failed

**Action**: Clear local costmap (reset to unknown), retry with fresh perception

**Use case**: Sensor noise caused false obstacles (e.g., sunlight glare on camera)

**Configuration**:
```yaml
# In BT XML
<ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
```

### Recovery Failure → Goal Failure

If all recoveries fail (spin + backup + clear costmap), Nav2 aborts goal:

```python
# In result_callback:
result = future.result().result
status = future.result().status

if status != GoalStatus.STATUS_SUCCEEDED:
    self.get_logger().error(f'Navigation failed with status: {status}')
    # Status codes:
    # - ABORTED (4): Recoveries failed, can't reach goal
    # - CANCELED (5): User canceled
```

**Humanoid-specific recovery** (custom behavior):
- Sit down to rest/reset balance
- Call for human assistance (teleoperation)
- Plan multi-step route (e.g., backtrack to known safe area, replan)

---

## 11. Full Navigation Workflow

Let's execute an end-to-end navigation task in Isaac Sim.

### Scenario: Office Navigation (10m Goal)

**Setup**:
- Isaac Sim office environment (from Chapter 1)
- Humanoid robot with cameras, LiDAR (Chapter 1 sensors)
- Isaac ROS Visual SLAM running (Chapter 2 odometry + map)
- Nav2 configured (this chapter)

**Goal**: Navigate from desk to conference room (10m away, through hallway with dynamic obstacles)

### Step 1: Launch Full Stack

```bash
# Terminal 1: Isaac Sim (simulation + sensors)
isaac-sim.sh

# Terminal 2: Isaac ROS Visual SLAM (perception)
ros2 launch isaac_vslam_humanoid.launch.py

# Terminal 3: Nav2 Navigation Stack
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/path/to/nav2_params.yaml

# Terminal 4: RViz2 Visualization
rviz2 -d /path/to/nav2_config.rviz
```

### Step 2: Set Navigation Goal (RViz2)

1. In RViz2, click **2D Nav Goal** button (toolbar)
2. Click on map at conference room location (10m away)
3. Drag arrow to set desired orientation (facing conference table)
4. Release → Goal sent to Nav2

**Alternatively, send programmatically**:
```python
navigator.navigate_to_goal(x=10.0, y=2.0, yaw=0.0)
```

### Step 3: Observe Navigation (RViz2 View)

**Global Costmap** (large blue/white grid):
- White = free space
- Dark gray = obstacles (walls, furniture)
- Blue = unknown (not yet mapped)
- Green line = global path (A* or Smac Planner)

**Local Costmap** (small colorful grid following robot):
- Updates in real-time as robot moves
- Rainbow colors = cost gradient (red = high cost near obstacles)
- Robot footprint = magenta polygon

**Robot Trajectory** (red line):
- Actual path taken (deviates slightly from planned path due to local avoidance)

**Dynamic Obstacle Appears** (person walks across path):
- Local costmap darkens where person is detected
- Robot slows down, curves around person
- Path smoothly adjusts (no jerky motions)

### Step 4: Goal Reached

After ~40 seconds (10m at 0.3 m/s average with obstacle avoidance):

```
[controller_server]: Goal reached!
[bt_navigator]: NavigateToPose succeeded
```

**Humanoid stops at conference room entrance**, facing table (as specified by goal orientation).

### Visualization Checklist

In RViz2, you should see:
- ✅ **TF tree**: `map` → `odom` → `base_link` → `camera_optical`, all frames connected
- ✅ **Global costmap**: Static obstacles (walls, desks) correctly marked
- ✅ **Local costmap**: Updating at 10 Hz, showing dynamic obstacles
- ✅ **Global path** (green): Smooth curve from start to goal
- ✅ **Local trajectory** (red): Robot's actual path, closely following green path
- ✅ **Robot footprint** (magenta): Aligned with robot model, not colliding with obstacles
- ✅ **Feature tracks** (green dots on camera image): Isaac ROS features being tracked

---

## 12. Safety for Physical Hardware

Simulation testing is safe; real hardware deployment requires caution.

### Simulation-First Workflow

**Never skip simulation testing**:

```
Mandatory Steps Before Hardware:
1. [Simulation] Test navigation in Isaac Sim (Chapter 1 environment)
2. [Simulation] Verify costmaps (no false obstacles, proper inflation)
3. [Simulation] Test obstacle avoidance (add dynamic obstacles in sim)
4. [Simulation] Test recoveries (artificially block robot, ensure recovery works)
5. [Simulation] 100% goal success rate over 10+ navigation tasks
─────────────────────────────────────────────────────────────
Only after passing all simulation tests:
6. [Hardware] Deploy to real humanoid in controlled environment
```

### Safety Zones (Virtual Barriers)

Prevent humanoid from entering dangerous areas (stairs, off-limits zones):

```yaml
# Add "keepout" layer to costmap
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_layer"]

      keepout_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /keepout_map  # Publish map with "forbidden" zones marked as obstacles
        # Mark stairs, drop-offs, restricted areas as permanent obstacles
```

**Create keepout map**:
1. Load SLAM map in image editor
2. Draw black (obstacle) over forbidden zones
3. Publish as `/keepout_map` topic

### Emergency Stop Integration

**Hardware e-stop** (physical button on humanoid):
- Wired directly to motor controllers (cuts power immediately)
- Should **not** rely on software

**Software kill switch** (for remote control):

```python
import rclpy
from std_srvs.srv import Empty

class EStopNode(Node):
    def __init__(self):
        super().__init__('estop_node')
        self.estop_client = self.create_client(Empty, '/stop_all_navigation')

    def trigger_estop(self):
        # Stop Nav2
        request = Empty.Request()
        self.estop_client.call_async(request)

        # Also stop motors (publish zero velocity)
        from geometry_msgs.msg import Twist
        stop_cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
```

### Gradual Deployment Strategy

**Phase 1: Stationary Testing** (robot doesn't move)
- Verify sensors publish (camera, LiDAR, IMU)
- Check Isaac ROS odometry (should be [0, 0, 0] if stationary)
- Test Nav2 launch (all nodes start without errors)

**Phase 2: Tethered Low-Speed** (robot on safety tether, max speed 0.1 m/s)
- Send 1m navigation goals
- Human holds tether (can stop robot physically if needed)
- Verify collision avoidance works (place obstacles in path)

**Phase 3: Untethered Medium-Speed** (max speed 0.3 m/s)
- 5m navigation goals in open space
- No tether, but human nearby with e-stop remote
- Test recoveries (block robot, verify it backs up correctly)

**Phase 4: Full-Speed Autonomous** (max speed 0.5 m/s)
- 10+ meter navigation goals
- Complex environments (hallways, furniture)
- Dynamic obstacles (people walking)

**Never skip phases**. Each phase proves safety before increasing risk.

---

## Summary and Module Completion

Congratulations! You've mastered **Nav2 for humanoid robots** and completed Module 3!

### What You Accomplished in Chapter 3

- ✅ Installed and configured Nav2 packages for humanoid navigation
- ✅ Configured global and local costmaps using Isaac ROS perception data (point clouds, odometry)
- ✅ Understood Nav2 behavior tree architecture and key plugins (planners, controllers, recoveries)
- ✅ Configured bipedal-specific constraints (step height, kinematic limits, balance considerations)
- ✅ Sent navigation goals using Python action clients and monitored progress
- ✅ Compared path planning algorithms (A*, Smac Planner, Theta*) and selected best for humanoids
- ✅ Implemented local trajectory planning (DWB controller) with obstacle avoidance
- ✅ Handled dynamic obstacles through real-time costmap updates and replanning
- ✅ Configured recovery behaviors (spin, backup, clear costmap) for stuck situations
- ✅ Executed full navigation workflow in Isaac Sim (goal → plan → avoid obstacles → reach goal)
- ✅ Applied safety considerations for real hardware deployment (simulation-first, e-stop, gradual rollout)

### Key Takeaways from Module 3

**Full Stack Integration**:
1. **Isaac Sim** (Chapter 1) provides the **simulation environment** and **synthetic training data**
2. **Isaac ROS** (Chapter 2) delivers **real-time perception** (odometry, point clouds) with GPU acceleration
3. **Nav2** (Chapter 3) enables **autonomous navigation** (path planning, obstacle avoidance)

**Humanoid-Specific Challenges Addressed**:
- **Step height limits**: Costmap filtering (&lt;20cm traversable)
- **Turning radius**: Smac Planner respects kinematic constraints
- **Balance**: Velocity/acceleration limits in controller configuration
- **Recovery**: Modified behaviors (slow spin, short backup) for stability

**Sim-to-Real Transfer**:
- Test extensively in Isaac Sim (100+ navigation tasks)
- Use gradual deployment (tethered → low speed → full speed)
- Always have hardware e-stop ready
- Validate safety zones (keepout maps) before autonomous operation

### What's Next?

You've completed **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**! You now have the skills to:

- Simulate humanoid robots in photorealistic environments (Isaac Sim)
- Implement real-time perception pipelines (Isaac ROS Visual SLAM)
- Enable autonomous navigation (Nav2 path planning and control)

**Potential Next Steps**:
1. **Apply to Your Project**: Integrate Isaac Sim + Isaac ROS + Nav2 for your own humanoid robot
2. **Advanced Topics**:
   - Whole-body motion planning (MoveIt 2 integration)
   - Reinforcement learning for locomotion (Isaac Gym)
   - Multi-robot coordination (Fleet management)
3. **Hardware Deployment**: Deploy your navigation stack to a real humanoid platform (Agility Digit, Boston Dynamics Spot, custom builds)

**Congratulations on completing Module 3!** You're now equipped to build intelligent, autonomous humanoid robots. 🤖

---

**Chapter Navigation**:

- ← Previous: [Chapter 2: Isaac ROS](/docs/module-3-isaac/isaac-ros)
- ↑ Back to Module 3: [Module 3 Index](/docs/module-3-isaac/)

---

*Chapter 3 of Module 3: The AI-Robot Brain (NVIDIA Isaac™)*
