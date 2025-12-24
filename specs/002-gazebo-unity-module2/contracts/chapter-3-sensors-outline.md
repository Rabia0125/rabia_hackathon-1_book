# Chapter 3 Outline: Sensor Simulation - Virtual Perception

**File**: `frontend_book/docs/module-2-simulation/03-sensor-simulation.md`
**Estimated Length**: ~8500 words
**Code Examples**: 8-9
**Diagrams**: 3

---

## Frontmatter

```yaml
---
sidebar_position: 3
slug: 03-sensor-simulation
title: "Sensor Simulation: LiDAR, Cameras, and IMUs"
sidebar_label: "Sensor Simulation"
description: "Learn to simulate LiDAR, depth cameras, and IMUs in Gazebo and Unity for complete perception system testing without hardware."
tags:
  - sensors
  - lidar
  - camera
  - imu
  - perception
---
```

---

## Section 1: Introduction - Why Simulate Sensors? (~600 words)

**Purpose**: Motivate sensor simulation for perception development

**Content**:
- The cost and time of hardware sensor testing
- Advantages of virtual sensors: perfect repeatability, no hardware damage, rapid prototyping
- What sensors humanoid robots need: LiDAR (obstacle avoidance), Depth cameras (vision), IMUs (balance)
- Sim-to-real gap: what transfers and what doesn't
- What readers will learn

**Callout**: Prerequisites
- Chapters 1-2 (Gazebo and Unity setup)
- Understanding of ROS 2 topics (Module 1, Chapter 2)

---

## Section 2: Sensor Fundamentals (~900 words)

**Purpose**: Explain each sensor type before simulation

**Content**:

### LiDAR (Light Detection and Ranging)
- How it works: laser pulses, time-of-flight
- Output: point cloud (3D coordinates)
- Parameters: range, resolution, scan rate, field of view
- Use cases: navigation, obstacle detection, mapping

### Depth Camera
- How it works: stereo vision or structured light
- Output: depth image (2D array of distances)
- Parameters: resolution, FOV, min/max range
- Use cases: grasping, person detection, 3D reconstruction

### IMU (Inertial Measurement Unit)
- Components: accelerometer, gyroscope, magnetometer
- Output: linear acceleration, angular velocity, orientation
- Parameters: noise, bias, update rate
- Use cases: balance control, odometry, state estimation

**Diagram**: Sensor data flow (Mermaid)
- Show: Physical World → Sensor → ROS 2 Topic → Processing Node → Action

---

## Section 3: LiDAR Simulation in Gazebo (~1200 words)

**Purpose**: Add and configure virtual LiDAR

**Content**:
- Gazebo sensor plugins (libgazebo_ros_ray_sensor.so)
- Adding LiDAR to URDF model
- Configuring scan parameters (horizontal FOV, samples, range)
- Visualizing point clouds in RViz2
- Performance impact of high-resolution scans

**Code Example 1**: LiDAR sensor in URDF
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="gpu_ray">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

**Code Example 2**: Subscribing to LiDAR data (Python)
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        # Process scan data
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')

def main():
    rclpy.init()
    node = LiDARProcessor()
    rclpy.spin(node)
```

---

## Section 4: Depth Camera Simulation in Gazebo (~1100 words)

**Purpose**: Simulate RGB-D cameras

**Content**:
- Gazebo depth camera plugin
- Adding depth camera to humanoid head
- RGB + Depth output (two topics)
- Configuring resolution, FOV, clip distances
- Visualizing depth images in RViz2 or image tools

**Code Example 3**: Depth camera in URDF
```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>20</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Code Example 4**: Processing depth images (Python)
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            10
        )

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Process depth: find nearest object
        min_depth = np.nanmin(depth_image)
        self.get_logger().info(f'Nearest depth: {min_depth:.2f}m')
```

---

## Section 5: IMU Simulation in Gazebo (~1000 words)

**Purpose**: Simulate inertial sensing for balance

**Content**:
- Gazebo IMU plugin
- Attaching IMU to robot torso
- Configuring noise models (Gaussian noise for accel/gyro)
- Understanding IMU frame conventions
- Using IMU data for orientation estimation

**Code Example 5**: IMU sensor in URDF
```xml
<gazebo reference="torso_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
    <imu>
      <noise>
        <type>gaussian</type>
        <rate>
          <mean>0.0</mean>
          <stddev>0.0001</stddev>
        </rate>
        <accel>
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </accel>
      </noise>
    </imu>
  </sensor>
</gazebo>
```

**Code Example 6**: Reading IMU data (Python)
```python
from sensor_msgs.msg import Imu

class IMUMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Extract orientation quaternion
        q = msg.orientation
        # Extract angular velocity
        w = msg.angular_velocity
        # Extract linear acceleration
        a = msg.linear_acceleration
        self.get_logger().info(f'Roll rate: {w.x:.2f} rad/s')
```

---

## Section 6: Sensors in Unity (~1300 words)

**Purpose**: Implement same sensors in Unity environment

**Content**:

### LiDAR in Unity
- Unity doesn't have built-in LiDAR (use Raycasting)
- Custom LiDAR script: shoot rays in circular pattern
- Publish point cloud to ROS 2
- Performance: fewer rays than real LiDAR (game engine limitation)

**Code Example 7**: Custom LiDAR in Unity (C#)
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class UnityLiDAR : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 10.0f;
    public float scanRate = 10.0f;

    private float[] ranges;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>("scan");
        ranges = new float[numRays];
        InvokeRepeating("PublishScan", 0f, 1f / scanRate);
    }

    void PublishScan()
    {
        for (int i = 0; i < numRays; i++)
        {
            float angle = (i / (float)numRays) * 360f;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = float.PositiveInfinity;
            }
        }

        // Create and publish LaserScan message
        LaserScanMsg msg = new LaserScanMsg();
        msg.ranges = ranges;
        ros.Publish("scan", msg);
    }
}
```

### Depth Camera in Unity
- Unity Camera component with depth texture
- Use RenderTexture for depth
- Publish as ROS 2 Image message

**Code Example 8**: Depth camera publisher (C#)
```csharp
public class UnityDepthCamera : MonoBehaviour
{
    private Camera depthCamera;
    private RenderTexture depthTexture;
    private ROSConnection ros;

    void Start()
    {
        depthCamera = GetComponent<Camera>();
        depthCamera.depthTextureMode = DepthTextureMode.Depth;

        depthTexture = new RenderTexture(640, 480, 24);
        depthCamera.targetTexture = depthTexture;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("camera/depth/image_raw");
    }

    void Update()
    {
        // Read depth texture and publish
        // Convert Unity depth to ROS Image message
    }
}
```

### IMU in Unity
- Use Articulation Body's velocity and acceleration
- Add simulated noise
- Publish sensor_msgs/Imu

---

## Section 7: Sensor Noise and Realism (~900 words)

**Purpose**: Make simulated sensors match real-world behavior

**Content**:
- Why noise matters: algorithms must handle imperfect data
- Types of noise: Gaussian, bias, drift
- Configuring noise parameters
- Sim-to-real gap: simulated sensors are "too perfect"
- Adding realistic noise in post-processing

**Callout**: Tip
- Start with low noise for algorithm development
- Gradually increase noise to match real sensor specs
- Test with worst-case noise before hardware deployment

**Diagram**: Sensor noise model (Mermaid)
- Show: Perfect Measurement → + Gaussian Noise → + Bias → + Drift → Realistic Output

---

## Section 8: Sensor Placement on Humanoid (~800 words)

**Purpose**: Teach where to mount sensors

**Content**:
- LiDAR placement: chest height for obstacle detection
- Depth camera placement: head for human-eye perspective
- IMU placement: torso center of mass for accurate orientation
- Multiple sensors: sensor fusion considerations
- Occlusion and field-of-view analysis

**Diagram**: Humanoid sensor placement (Mermaid or ASCII)
- Show side view of humanoid with sensors labeled

---

## Section 9: Complete Perception Pipeline (~1200 words)

**Purpose**: End-to-end example integrating all sensors

**Content**:
- Scenario: Humanoid navigates to human using sensors
- Sensors: LiDAR (obstacles), Depth camera (human detection), IMU (balance)
- ROS 2 nodes:
  1. Sensor data subscribers
  2. Obstacle avoidance (from LiDAR)
  3. Human tracking (from camera)
  4. Balance controller (from IMU)
  5. Motion planner (combines all)

**Code Example 9**: Sensor fusion node (Python)
```python
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to all sensors
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.depth_sub = self.create_subscription(Image, 'camera/depth/image_raw', self.depth_cb, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_cb, 10)

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def lidar_cb(self, msg):
        # Detect obstacles
        pass

    def depth_cb(self, msg):
        # Detect humans
        pass

    def imu_cb(self, msg):
        # Monitor balance
        pass

    def fuse_and_navigate(self):
        # Combine sensor data and decide action
        pass
```

---

## Section 10: Gazebo vs Unity for Sensors (~600 words)

**Purpose**: Comparison table for sensor simulation

**Comparison Table**:

| Sensor | Gazebo | Unity |
|--------|--------|-------|
| LiDAR | Native plugin, accurate | Raycast-based, less accurate |
| Depth Camera | Native plugin, good | Good with shaders |
| IMU | Native plugin, accurate | Requires custom script |
| Performance | Sensor-heavy, slower | Graphics-heavy, faster |
| Ease of use | Plugin configuration (XML) | Script-based (C#) |
| Best for | Algorithm validation | Visual demos |

**Recommendation**: Use Gazebo for sensor algorithm development, Unity for visualization and human interaction scenarios.

---

## Section 11: Summary / Key Takeaways

**Key Points**:
- Virtual sensors enable perception algorithm testing without hardware
- LiDAR, depth cameras, and IMUs are essential for humanoid robots
- Gazebo provides accurate sensor plugins with configurable parameters
- Unity requires custom scripts but integrates with ROS 2 seamlessly
- Sensor noise models improve sim-to-real transfer
- Sensor fusion combines multiple modalities for robust perception
- Start with Gazebo for development, use Unity for demonstrations

**Next Steps**:
You've completed Module 2! You can now create physics-based simulations (Gazebo), photorealistic digital twins (Unity), and equip robots with virtual sensors. Next: Module 3 (coming soon) will cover computer vision algorithms for humanoid perception.

---

## Section 12: Self-Assessment Questions

**Question 1** (Easy):
What type of data does a LiDAR sensor output?
- A) RGB image
- B) Point cloud (3D coordinates)
- C) Joint angles
- D) Audio waveform
**Answer**: B

**Question 2** (Easy):
Why add noise to simulated sensors?
**Answer**: To match real-world sensor behavior and ensure algorithms handle imperfect data, improving sim-to-real transfer.

**Question 3** (Medium):
Your simulated LiDAR shows perfect circles at 10m range, but the real LiDAR has gaps and noise. How do you make simulation more realistic?
**Answer**: Add Gaussian noise to range measurements, configure bias and drift parameters in the sensor plugin, and test with worst-case noise levels matching the real sensor's datasheet.

**Question 4** (Medium):
You want to test a person-following algorithm. Should you use Gazebo or Unity for sensor simulation?
**Answer**: Use both: Develop obstacle avoidance in Gazebo (accurate LiDAR), then validate human tracking in Unity (photorealistic depth camera + human avatars).

**Question 5** (Hard):
Your depth camera in simulation works perfectly, but on the real robot it fails in bright sunlight. Explain why and how to improve simulation.
**Answer**: Simulation doesn't model real-world lighting effects (glare, saturation, infrared interference). To improve: (1) Test algorithms with missing/invalid depth data (simulate failures), (2) Add camera exposure simulation in Unity, (3) Use multiple sensor modalities (LiDAR + camera) for robustness.

---

## Additional Resources

- [Gazebo Sensor Plugins](https://gazebosim.org/api/gazebo/6.0/sensors.html)
- [ROS 2 sensor_msgs Documentation](https://docs.ros2.org/latest/api/sensor_msgs/)
- [Unity Raycasting](https://docs.unity3d.com/ScriptReference/Physics.Raycast.html)
- [Sensor Fusion Tutorials](http://wiki.ros.org/sensor_fusion)
