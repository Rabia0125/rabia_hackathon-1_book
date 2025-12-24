# Research: Simulation Best Practices for Educational Content

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-24
**Purpose**: Document research findings and design decisions for teaching simulation to AI/robotics students

---

## 1. Gazebo vs Unity Selection Criteria

### Decision: Use Both (Complementary Approach)

**Rationale**:
- Gazebo excels at physics-accurate simulation for algorithm development
- Unity excels at photorealistic rendering for demonstrations and human interaction
- Teaching both provides complete skillset for industry/research

**Alternatives Considered**:
1. **Gazebo Only**: Rejected because students need visual fidelity for demos and user studies
2. **Unity Only**: Rejected because Unity's physics is game-engine focused, less accurate for robotics
3. **Isaac Sim (NVIDIA)**: Rejected due to steep learning curve and GPU requirements beyond free-tier

**Supporting Evidence**:
- ROS 2 documentation recommends Gazebo for physics testing
- Unity Robotics Hub has 2000+ GitHub stars, active community
- Industry uses Gazebo for R&D, Unity for visualization (e.g., warehouse robotics demos)

---

## 2. Chapter Sequence Decision

### Decision: Gazebo (Ch1) → Unity (Ch2) → Sensors (Ch3)

**Rationale**:
- **Physics First**: Students must understand physics engines before visual rendering
- **Sensors Last**: Requires understanding of both simulation environments
- **Progressive Complexity**: Each chapter builds on previous

**Alternatives Considered**:
1. **Unity First**: Rejected because students need physics foundation before graphics
2. **Parallel Gazebo/Unity**: Rejected because comparing without experiencing both causes confusion
3. **Sensors in Each Chapter**: Rejected to avoid repetition; consolidated approach is clearer

**Pedagogical Support**:
- Bloom's Taxonomy: Knowledge (Ch1) → Application (Ch2) → Synthesis (Ch3)
- Cognitive load theory: Introduce one complex system at a time

---

## 3. Code Example Pattern

### Decision: Complete, Runnable Examples with Line-by-Line Explanation

**Pattern Structure**:
```
1. Conceptual explanation (what and why)
2. Complete code block (copy-paste ready)
3. Line-by-line walkthrough
4. Expected output
5. Common errors and fixes
```

**Rationale**:
- Module 1 pattern was successful (student feedback: "code just works")
- Self-contained examples reduce frustration
- Explanation after code allows experimentation then understanding

**Alternatives Considered**:
1. **Snippet-Only**: Rejected because students struggle with missing imports/setup
2. **Explanation-First**: Rejected because students want to "try it first" (active learning)
3. **Video Tutorials**: Rejected because text is searchable and copy-pasteable

**Research Support**:
- "Worked examples effect" (Sweller): Complete solutions accelerate learning
- Active learning: Students learn by doing, then reflecting

---

## 4. Gazebo Version Selection

### Decision: Target Gazebo Fortress (or Harmonic)

**Rationale**:
- Gazebo Fortress is LTS with ROS 2 Humble (most common stack)
- Gazebo Harmonic is latest stable (future-proof)
- "Gazebo Classic" (Gazebo 11) is deprecated, avoid confusing students

**Alternatives Considered**:
1. **Gazebo Classic**: Rejected because it's deprecated and ROS 1-focused
2. **Gazebo Garden**: Rejected because Fortress LTS is more stable
3. **Custom Physics Engine**: Rejected due to complexity and reinventing wheel

**Technical Considerations**:
- SDF format (vs URDF): Gazebo uses SDF internally, but accepts URDF
- Plugin API: Fortress/Harmonic have modern plugin system
- ROS 2 integration: gz_ros2_control is mature for Fortress

---

## 5. Unity Version Selection

### Decision: Unity 2022.3 LTS

**Rationale**:
- LTS (Long Term Support) = stable for 2+ years
- Unity Robotics Hub officially supports 2022 LTS
- URP (Universal Render Pipeline) is default, good performance

**Alternatives Considered**:
1. **Unity 2023.x**: Rejected because not LTS, may have breaking changes
2. **Unity 2021 LTS**: Rejected because 2022 LTS is current standard
3. **Unreal Engine**: Rejected due to steeper learning curve and less ROS 2 integration

**Technical Considerations**:
- Unity Robotics Hub compatibility: 2022.3 LTS fully supported
- Articulation Bodies: Mature in 2022.3 (introduced in 2020.1, refined since)
- Performance: URP optimized for real-time simulation

---

## 6. Sensor Simulation Approach

### Decision: Gazebo Plugins for Accuracy, Unity Raycasting for Visualization

**Gazebo Sensor Strategy**:
- Use native plugins (libgazebo_ros_ray_sensor, libgazebo_ros_camera, libgazebo_ros_imu)
- Configure via URDF `<gazebo>` tags
- Publish to standard ROS 2 message types (sensor_msgs)

**Unity Sensor Strategy**:
- LiDAR: Custom raycast script (Unity has no native LiDAR)
- Depth Camera: Unity Camera with depth texture
- IMU: Articulation Body velocity/acceleration + noise

**Rationale**:
- Gazebo plugins are research-grade, Unity is approximation
- Students learn "real" sensor simulation in Gazebo
- Unity sensors sufficient for visualization and demos

**Alternatives Considered**:
1. **Unity Only**: Rejected because raycasts don't match real LiDAR behavior
2. **Gazebo Only**: Rejected because Unity visualization is needed
3. **ROS 2 Sensor Plugins in Unity**: Rejected because underdeveloped community plugins

**Research Finding**:
- Academic papers use Gazebo for sensor algorithm validation
- Industry uses Unity for customer demos and HMI testing

---

## 7. Diagram Strategy

### Decision: Mermaid Diagrams (Inline) for Architecture, Optional PNG for 3D Visuals

**Rationale**:
- Mermaid renders natively in Docusaurus
- Version-controllable (text, not binary)
- Easy to update without external tools
- Consistent with Module 1's approach

**Diagram Types**:
- Architecture diagrams: Mermaid (boxes and arrows)
- Flow charts: Mermaid (decision trees)
- 3D visualizations: PNG (if needed, but prefer screenshots from Gazebo/Unity)

**Alternatives Considered**:
1. **All PNG Images**: Rejected because harder to maintain and version control
2. **draw.io/Lucidchart**: Rejected because requires external tool
3. **No Diagrams**: Rejected because visual learners need architecture diagrams

---

## 8. Performance Optimization Teaching

### Decision: Teach Performance from Chapter 1, Reinforce Throughout

**Approach**:
- Chapter 1 (Gazebo): Introduce real-time factor, physics timestep, collision simplification
- Chapter 2 (Unity): Focus on FPS, profiling, LOD, occlusion culling
- Chapter 3 (Sensors): Address sensor count vs performance trade-offs

**Rationale**:
- Performance issues frustrate beginners (simulation runs slowly)
- Proactive teaching prevents "my simulation is too slow" questions
- Real-time requirements are critical for robotics (vs offline batch processing)

**Pedagogical Pattern**:
1. Explain metric (real-time factor, FPS)
2. Show how to measure (Gazebo GUI, Unity Profiler)
3. Teach optimization (simplify meshes, reduce sensor rate)
4. Provide target benchmarks (Gazebo: 1.0x RTF, Unity: 30 FPS minimum)

---

## 9. Gazebo vs Unity Comparison Framework

### Decision: Dedicated Comparison Section in Chapter 2

**Structure**:
- Comparison table (side-by-side features)
- Decision framework: "Use Gazebo when..." / "Use Unity when..."
- Example workflow: "Use both" (Gazebo for dev, Unity for demo)

**Rationale**:
- Students ask "which should I use?" - answer proactively
- Comparison after experiencing both is more meaningful
- Reinforces that tools are complementary, not competitive

**Comparison Dimensions**:
- Physics fidelity
- Visual quality
- Performance
- Learning curve
- ROS 2 integration
- Community support
- Best use cases

---

## 10. System Requirements Approach

### Decision: Specify Requirements, Provide Alternatives

**Requirements Disclosure**:
- Chapter 1: Gazebo system requirements (GPU for rendering, CPU for physics)
- Chapter 2: Unity system requirements (GPU-intensive)
- Provide alternatives: Docker containers, cloud VMs (AWS/GCP free tier), headless mode

**Rationale**:
- Transparency prevents frustration ("why won't this run?")
- Alternatives ensure accessibility (students with low-end hardware)
- Matches Module 1 pattern (assumes readers follow installation guides)

**Edge Case Handling**:
- Students without GPU → Gazebo headless mode, Unity lowered graphics settings
- Windows users → WSL2 for Gazebo, native Unity
- Mac users → Docker for Gazebo (no native support), Unity native

---

## 11. Human-Robot Interaction Content

### Decision: Dedicated Section in Chapter 2 (Unity-Specific)

**Content Focus**:
- Importing human avatars (Mixamo, Ready Player Me)
- Scripting basic interactions (following, approaching, gesturing)
- Social robotics scenarios

**Rationale**:
- Human interaction is Unity's unique strength (vs Gazebo)
- Growing field (social robotics, service robots, elder care)
- Prepares students for UX testing and user studies

**Scope Limit**:
- Basic interactions only (not full HMI or VR)
- Focus on robot behavior, not human AI
- Animation: Use pre-made animations (not custom rigging)

---

## 12. Sim-to-Real Gap Discussion

### Decision: Address Throughout, Emphasize in Sensor Chapter

**Approach**:
- Chapter 1: Mention that simulation physics != real-world physics
- Chapter 2: Visual fidelity doesn't equal physical fidelity
- Chapter 3: Sensor noise models are approximations

**Rationale**:
- Students transitioning to hardware need realistic expectations
- Proactive discussion prevents "why doesn't this work on the real robot?" frustration
- Teaching mindset: simulation for prototyping, hardware for validation

**Best Practice Teaching**:
- Always validate on hardware before deployment
- Use simulation for rapid iteration
- Add realistic noise/disturbances in simulation
- Test edge cases that are dangerous/expensive in hardware

---

## Summary of Design Decisions

| Decision | Rationale | Alternative Rejected |
|----------|-----------|---------------------|
| Use Both Gazebo & Unity | Complementary strengths | Gazebo-only, Unity-only |
| Physics → Rendering → Sensors sequence | Progressive complexity | Parallel, sensors-first |
| Complete code examples | Worked examples effect | Snippets, explanation-first |
| Gazebo Fortress/Harmonic | LTS + ROS 2 Humble | Gazebo Classic, custom |
| Unity 2022.3 LTS | Stability + Robotics Hub support | Unity 2023, Unreal |
| Mermaid diagrams | Version control + Docusaurus native | PNG images, external tools |
| Performance early & often | Prevents frustration | Performance as afterthought |
| Comparison in Ch2 | After experiencing both | Comparison upfront |
| Docker/cloud alternatives | Accessibility | Hardware requirements only |
| HRI in Unity only | Unity's unique strength | HRI in both, or omit |
| Sim-to-real discussion throughout | Realistic expectations | Omit or end-only |

---

## References

1. Gazebo Official Documentation: https://gazebosim.org/docs
2. Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
3. ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
4. Sweller, J. (1988). Cognitive load during problem solving: Effects on learning.
5. Active Learning in STEM Education (Freeman et al., 2014)

---

This research document guided all design decisions for Module 2 content structure, tooling choices, and pedagogical approach.
