# Implementation Plan: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

**Branch**: `003-isaac-ai-brain` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-isaac-ai-brain/spec.md` + User request to add Module 3 to Docusaurus with 3 chapter pages as Markdown files

## Summary

Add Module 3 ("The AI-Robot Brain - NVIDIA Isaac™") to the Docusaurus documentation structure following the established patterns from Module 1 and Module 2. Create 4 Markdown files (1 index + 3 chapters) covering Isaac Sim, Isaac ROS, and Nav2 for humanoid robots. Update sidebars.ts configuration to include the new module in the navigation structure.

**Technical Approach**: Follow existing Docusaurus patterns observed in Module 1 and Module 2, creating a new `module-3-isaac` directory under `frontend_book/docs/` with index.md and three numbered chapter files (01-isaac-sim.md, 02-isaac-ros.md, 03-nav2-humanoids.md). Content will be developer-focused, maintaining the established pedagogical structure with learning objectives, practical examples, and the consistent chapter progression pattern.

## Technical Context

**Language/Version**: Markdown (Docusaurus-flavored with JSX support), TypeScript 5.x (for configuration)
**Primary Dependencies**: Docusaurus v4 (future flag enabled), React 18 for JSX components
**Storage**: Static Markdown files in `frontend_book/docs/module-3-isaac/`
**Testing**: Manual review of rendered Docusaurus output, link validation, frontmatter validation
**Target Platform**: Static site generation → GitHub Pages deployment
**Project Type**: Documentation site (Docusaurus)
**Performance Goals**: Fast page load (<2s), SEO-optimized Markdown structure
**Constraints**: Must follow existing Module 1 & Module 2 patterns, maintain consistent sidebar navigation structure
**Scale/Scope**: 4 new Markdown files (~3000-4000 words total), 1 TypeScript configuration update

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Spec-Driven Reproducibility
**Status**: PASS
- Specification created via `/sp.specify` (specs/003-isaac-ai-brain/spec.md)
- Implementation plan follows `/sp.plan` workflow
- All requirements traceable to spec FR-001 through FR-031

### ✅ III. Developer-Focused Writing
**Status**: PASS
- Target audience: AI and robotics developers (defined in spec)
- Content will include technical precision, practical examples, code snippets
- Follows Module 1 & Module 2 pedagogical patterns (learning objectives, prerequisites, hands-on examples)

### ✅ V. Public Reproducibility
**Status**: PASS
- Docusaurus platform (static site generation)
- Markdown-based authoring (spec.md:lines 1-183 defines clear requirements)
- Deployable to GitHub Pages (zero cost)

### N/A: II. Content Accuracy (RAG Chatbot)
**Status**: N/A - This phase only creates documentation structure
- RAG indexing will occur after content is written (constitution:lines 92-97)
- Future task: Index Module 3 content for RAG retrieval

### N/A: IV. Retrieval Transparency (RAG Chatbot)
**Status**: N/A - RAG integration is out of scope for this feature

### N/A: VI. Test-Driven Quality
**Status**: DEFERRED - Manual validation planned
- Automated testing for Docusaurus build (npm run build)
- Manual review of rendered output
- Link validation for internal references
- No automated content quality tests at this stage

**Overall Constitution Compliance**: ✅ PASS (2 applicable checks passed, 4 N/A for RAG-specific requirements)

## Project Structure

### Documentation (this feature)

```text
specs/003-isaac-ai-brain/
├── spec.md                    # Feature specification (completed via /sp.specify)
├── plan.md                    # This file (/sp.plan command output)
├── checklists/
│   └── requirements.md        # Spec quality validation (completed)
└── tasks.md                   # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend_book/
├── docs/
│   ├── intro.md               # Existing introduction
│   ├── module-1-ros2/         # Existing Module 1
│   │   ├── index.md
│   │   ├── 01-ros2-fundamentals.md
│   │   ├── 02-python-ros-control.md
│   │   └── 03-humanoid-urdf.md
│   ├── module-2-simulation/   # Existing Module 2
│   │   ├── index.md
│   │   ├── 01-gazebo-simulation.md
│   │   ├── 02-unity-digital-twin.md
│   │   └── 03-sensor-simulation.md
│   └── module-3-isaac/        # NEW: Module 3 to be created
│       ├── index.md           # Module 3 overview and chapter navigation
│       ├── 01-isaac-sim.md    # Chapter 1: NVIDIA Isaac Sim
│       ├── 02-isaac-ros.md    # Chapter 2: Isaac ROS (VSLAM & Perception)
│       └── 03-nav2-humanoids.md # Chapter 3: Nav2 for Humanoids
├── sidebars.ts                # MODIFY: Add Module 3 navigation
├── docusaurus.config.ts       # NO CHANGE: Configuration already supports new modules
├── src/
│   ├── css/
│   │   └── custom.css         # NO CHANGE: Existing styles
│   └── pages/
│       └── index.tsx          # NO CHANGE: Landing page
└── package.json               # NO CHANGE: Dependencies already configured
```

**Structure Decision**: Following Docusaurus "Docs" plugin structure with category-based sidebar organization. Module 3 follows the established pattern from Modules 1-2: a directory with an index.md (module overview) and three numbered chapter files (01-, 02-, 03-). This maintains consistency and leverages Docusaurus's automatic sidebar generation via sidebars.ts configuration.

## Complexity Tracking

> **No Constitution violations detected - this section intentionally left empty**

All constitution checks passed or are appropriately deferred to later phases (RAG integration).

---

## Phase 0: Research and Discovery

### Objectives
1. Analyze existing Module 1 and Module 2 structure, frontmatter patterns, and content organization
2. Understand Docusaurus sidebar configuration for category-based navigation
3. Identify reusable content patterns (learning objectives, prerequisites, code blocks, callouts)
4. Define Module 3 content structure based on spec requirements (FR-001 through FR-031)

### Research Questions

#### Q1: What frontmatter structure is used in existing modules?
**Answer** (from module-1-ros2/index.md:1-5):
```yaml
---
sidebar_position: 0
title: "Module 1: The Robotic Nervous System"
description: Learn how ROS 2 acts as the nervous system of humanoid robots...
---
```

**Answer** (from module-1-ros2/01-ros2-fundamentals.md:1-12):
```yaml
---
sidebar_position: 1
slug: 01-ros2-fundamentals
title: "ROS 2 Fundamentals: The Robotic Nervous System"
sidebar_label: "ROS 2 Fundamentals"
description: "Learn how ROS 2 acts as the nervous system..."
tags:
  - ros2
  - fundamentals
  - architecture
  - nervous-system
---
```

**Pattern**: Index files use `sidebar_position: 0`, chapter files use `sidebar_position: 1/2/3`. Chapters include `slug`, `sidebar_label`, and `tags` for better navigation and SEO.

#### Q2: How is sidebar navigation configured?
**Answer** (from frontend_book/sidebars.ts:17-46):
```typescript
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        'module-1-ros2/ros2-fundamentals',
        'module-1-ros2/python-ros-control',
        'module-1-ros2/humanoid-urdf',
      ],
    },
    // Module 2 follows same pattern...
  ],
};
```

**Pattern**: Each module is a `category` with `link` to index.md and `items` array referencing chapter files without the `.md` extension or number prefix.

#### Q3: What content structure patterns exist in chapter files?
**Answer** (from module-1-ros2/01-ros2-fundamentals.md:14-35):
- **Section 1**: Learning Objectives (numbered list)
- **Section 2**: Prerequisites (using `:::info` callout)
- **Section 3**: Main content with numbered headings (## 1. Introduction, ## 2. Core Concepts, etc.)
- **Section 4**: Practical examples with code blocks
- **Section 5**: Summary/Next Steps

**Pattern**: Consistent pedagogical structure across all chapters with clear learning objectives upfront, prerequisite callouts, and progressive content disclosure.

#### Q4: What naming conventions are used for files and slugs?
**Answer**:
- **Directory names**: `module-N-shortname` (e.g., `module-1-ros2`, `module-2-simulation`)
- **Chapter files**: `0N-descriptive-name.md` (e.g., `01-ros2-fundamentals.md`, `02-python-ros-control.md`)
- **Slugs in frontmatter**: Match filename without number (e.g., `slug: 01-ros2-fundamentals`)
- **Sidebar items**: Reference without extension or path prefix within category (e.g., `'module-1-ros2/ros2-fundamentals'`)

**Decision for Module 3**:
- Directory: `module-3-isaac`
- Files: `index.md`, `01-isaac-sim.md`, `02-isaac-ros.md`, `03-nav2-humanoids.md`
- Slugs: `01-isaac-sim`, `02-isaac-ros`, `03-nav2-humanoids`

#### Q5: How are visual elements and callouts used?
**Answer** (patterns observed):
- **Tables**: Used for comparisons (e.g., module-1-ros2/index.md:25-31 - nervous system analogy table)
- **Callouts**: `:::info`, `:::tip`, `:::warning` for prerequisites, tips, and migration notes
- **Code blocks**: Fenced with language syntax highlighting (```python, ```bash, ```xml)
- **Diagrams**: ASCII art for learning paths (module-1-ros2/index.md:75-91)
- **Emojis**: Used sparingly for visual markers (✅, ❌, 🧠) in lists

**Pattern**: Visual elements enhance readability without overwhelming content. ASCII diagrams for workflow visualization.

### Research Summary

**Findings**:
1. **Consistent Structure**: All modules follow identical patterns (index + 3 chapters)
2. **Docusaurus Conventions**: Frontmatter with sidebar_position, title, description, tags
3. **Sidebar Configuration**: Category-based with explicit item arrays in sidebars.ts
4. **Content Pedagogy**: Learning objectives → Prerequisites → Main content → Examples → Next steps
5. **Naming**: Predictable, SEO-friendly naming (module-N-shortname, 0N-chapter-name)

**Constraints Identified**:
- Must maintain frontmatter consistency for Docusaurus build to succeed
- Sidebar items must reference correct paths (without .md, using directory prefixes)
- Chapter numbering starts at 01 (not 1) for alphabetical sorting
- Tags should be relevant for future search/filtering

**Risks**:
1. **Sidebar misconfiguration** → Broken navigation (mitigation: validate against existing patterns)
2. **Frontmatter errors** → Build failure (mitigation: copy-paste-modify from working modules)
3. **Broken internal links** → 404s (mitigation: use relative links, test with `npm run build`)

---

## Phase 1: Detailed Design

### Architecture Decisions

#### AD-1: Module 3 Directory Structure
**Decision**: Use `module-3-isaac` as directory name (not `module-3-isaac-ai-brain` or `module-3-nvidia-isaac`)

**Rationale**:
- **Brevity**: "isaac" is sufficient and recognizable in robotics community
- **Consistency**: Module 1 uses "ros2" (not "ros2-nervous-system"), Module 2 uses "simulation" (not "simulation-digital-twin")
- **URL-friendly**: Shorter directory name = cleaner URLs (e.g., `/docs/module-3-isaac/isaac-sim`)

**Alternatives Considered**:
- `module-3-nvidia-isaac` (rejected: "nvidia" redundant - Isaac is NVIDIA's brand)
- `module-3-isaac-ai-brain` (rejected: too long, "ai-brain" is metaphorical not technical)

**Impact**: All file paths, sidebar configuration, and internal links will reference `module-3-isaac/`

---

#### AD-2: Chapter File Naming
**Decision**:
- Chapter 1: `01-isaac-sim.md` (slug: `isaac-sim`)
- Chapter 2: `02-isaac-ros.md` (slug: `isaac-ros`)
- Chapter 3: `03-nav2-humanoids.md` (slug: `nav2-humanoids`)

**Rationale**:
- **Specificity**: "isaac-sim" and "isaac-ros" are precise technical terms (match NVIDIA's branding)
- **Searchability**: "nav2-humanoids" distinguishes from generic "navigation" (emphasizes bipedal focus)
- **Parallel Structure**: Mirrors spec.md chapter titles (FR-001 to FR-009 = Isaac Sim, FR-010 to FR-019 = Isaac ROS, FR-020 to FR-031 = Nav2)

**Alternatives Considered**:
- `01-simulation.md` (rejected: too generic, doesn't highlight Isaac Sim's unique features)
- `03-navigation.md` (rejected: doesn't emphasize humanoid-specific challenges)

**Impact**: Sidebar configuration must reference these exact slugs without the `01-`, `02-`, `03-` prefixes

---

#### AD-3: Content Organization within Chapters
**Decision**: Follow this structure for each chapter:

```markdown
---
[frontmatter]
---

# [Chapter Title]

## Learning Objectives
[Numbered list of outcomes - maps to spec.md Acceptance Scenarios]

## Prerequisites
:::info Before You Begin
[List of required knowledge from previous modules]
:::

---

## 1. Introduction: Why [Topic]?
[Motivation, real-world context, problem being solved]

## 2. Core Concepts
[Technical fundamentals with tables, diagrams, analogies]

## 3. Hands-On: [Practical Example]
[Step-by-step walkthrough with code/commands]

## 4. Advanced Topics
[Performance tuning, troubleshooting, best practices]

## 5. Summary and Next Steps
[Recap key concepts, link to next chapter]

---

**Chapter Navigation**:
- ← Previous: [Link]
- → Next: [Link]
- ↑ Back to Module 3 Overview: [Link]
```

**Rationale**:
- **Consistency**: Matches Module 1 & Module 2 patterns (observed in research phase)
- **Progressive Disclosure**: Starts with "why" (motivation), moves to "what" (concepts), then "how" (hands-on)
- **Traceability**: Learning objectives map directly to spec.md acceptance scenarios (FR-001 to FR-031)
- **Navigation**: Clear chapter links reduce user confusion

**Spec Alignment**:
- Chapter 1 Learning Objectives → spec.md FR-001 to FR-009 (Isaac Sim)
- Chapter 2 Learning Objectives → spec.md FR-010 to FR-019 (Isaac ROS)
- Chapter 3 Learning Objectives → spec.md FR-020 to FR-031 (Nav2)

---

#### AD-4: Module 3 Index Page Content
**Decision**: Include these sections in `index.md`:

1. **Module Overview** (maps to spec User Story priorities P1/P2/P3)
2. **Why Isaac for Humanoids?** (unique value proposition)
3. **Module Chapters** (3 chapter cards with learning outcomes)
4. **Learning Path Diagram** (ASCII art: Isaac Sim → Isaac ROS → Nav2)
5. **Prerequisites** (Module 1 + Module 2 required)
6. **Hardware Requirements** (NVIDIA GPU or cloud alternatives - addresses spec.md Edge Cases)

**Rationale**:
- **Orientation**: Developers need to understand why Isaac ecosystem matters (spec Success Criteria SC-007: articulate performance benefits)
- **Prerequisites**: Clear dependency on Module 1 & Module 2 (spec Dependencies D-001, D-002)
- **Hardware Callout**: Addresses spec Edge Case "developers without NVIDIA GPU" upfront (spec.md:63-64)

**Content Sources**:
- Overview → spec.md:10-24 (User Story 1 description)
- Prerequisites → spec.md:149-159 (Assumptions A-001 to A-010)
- Hardware → spec.md:63-64 (Edge Case handling)

---

#### AD-5: Sidebar Configuration Update
**Decision**: Add Module 3 to `sidebars.ts` after Module 2 with this structure:

```typescript
{
  type: 'category',
  label: 'Module 3: The AI-Robot Brain',
  link: {
    type: 'doc',
    id: 'module-3-isaac/index',
  },
  items: [
    'module-3-isaac/isaac-sim',
    'module-3-isaac/isaac-ros',
    'module-3-isaac/nav2-humanoids',
  ],
},
```

**Rationale**:
- **Consistency**: Exact same structure as Module 1 & Module 2 (sidebars.ts:20-46)
- **Category Link**: Points to index.md for module overview (not first chapter)
- **Item Paths**: Use directory prefix + slug (no `.md` extension, no number prefix)

**Validation**: After change, `npm run build` must succeed without warnings

---

### Data Model

**N/A for this feature** - No database entities, APIs, or complex data structures. All content is static Markdown.

---

### API Contracts / Interfaces

**N/A for this feature** - No programmatic APIs. Only human-readable Markdown content.

**Internal "Contract"** (Docusaurus frontmatter schema):

```yaml
# Required fields for all chapter pages:
sidebar_position: <number>    # 0 for index, 1/2/3 for chapters
slug: <string>                # URL-friendly identifier (no numbers)
title: <string>               # Full chapter title (appears in header)
sidebar_label: <string>       # Shorter label for sidebar navigation
description: <string>         # SEO meta description (50-160 chars)
tags: <array<string>>         # Categorization for search/filtering

# Optional but recommended:
---
# Body content follows...
```

**Enforcement**: Docusaurus build will fail if required fields are missing or malformed.

---

### File-by-File Breakdown

#### File 1: `frontend_book/docs/module-3-isaac/index.md`
**Purpose**: Module 3 landing page with overview, chapter navigation, prerequisites

**Content Sections** (estimated 800-1000 words):
1. **Frontmatter** (yaml):
   - `sidebar_position: 0`
   - `title: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"`
   - `description: "Learn high-fidelity simulation, perception, and navigation..."`
2. **Module Header**:
   - Title, duration (~4 hours), prerequisites (Module 1 + Module 2)
3. **Overview** (3 paragraphs):
   - What is Isaac ecosystem (Sim + ROS + Nav2)
   - Why it matters for humanoid robotics
   - What you'll learn by end of module
4. **Why Isaac for Humanoids?** (comparison table):
   - Isaac Sim vs generic simulators (photorealism, physics fidelity)
   - Isaac ROS vs standard ROS 2 (GPU acceleration, 2-5x speedup)
   - Nav2 for bipedal vs wheeled robots (constraints, foot placement)
5. **Module Chapters** (3 cards):
   - Chapter 1: Isaac Sim (learning outcomes from FR-001 to FR-009)
   - Chapter 2: Isaac ROS (learning outcomes from FR-010 to FR-019)
   - Chapter 3: Nav2 (learning outcomes from FR-020 to FR-031)
6. **Learning Path Diagram** (ASCII art):
   - Box 1: Isaac Sim (simulation + synthetic data)
   - Box 2: Isaac ROS (perception + VSLAM)
   - Box 3: Nav2 (navigation + path planning)
7. **Prerequisites**:
   - Module 1 (ROS 2, rclpy, URDF) - link to module-1-ros2/index
   - Module 2 (Gazebo, Unity) - link to module-2-simulation/index
8. **Hardware Requirements** (callout):
   - NVIDIA GPU (RTX series recommended)
   - Or cloud alternatives (AWS, GCP with GPU instances)
   - Disk space (~20GB for Isaac Sim)
9. **What's Next?**: Link to Chapter 1 (isaac-sim.md)

**Spec Traceability**:
- Overview → spec.md User Stories (lines 10-58)
- Prerequisites → spec.md Dependencies (lines 175-182) & Assumptions (lines 149-159)
- Hardware → spec.md Edge Cases (lines 63-64) & Assumptions A-003, A-007

---

#### File 2: `frontend_book/docs/module-3-isaac/01-isaac-sim.md`
**Purpose**: Chapter 1 - Teach Isaac Sim photorealistic simulation and synthetic data generation

**Content Sections** (estimated 2000-2500 words):
1. **Frontmatter**:
   - `sidebar_position: 1`
   - `slug: isaac-sim`
   - `title: "Isaac Sim: Photorealistic Simulation"`
   - `sidebar_label: "Isaac Sim"`
   - `tags: [isaac-sim, simulation, synthetic-data, domain-randomization]`
2. **Learning Objectives** (5-6 bullets mapping to spec FR-001 to FR-009)
3. **Prerequisites** (callout):
   - Module 1 Chapter 3 (URDF modeling)
   - Module 2 Chapter 1 (Gazebo basics)
   - NVIDIA GPU or cloud access
4. **Main Content**:
   - **1. Introduction: Why Isaac Sim?** (FR-001)
     - Sim-to-real gap problem
     - Photorealistic rendering vs basic visualization
     - Use cases: algorithm testing, dataset generation, demonstrations
   - **2. Installation and Setup** (FR-002)
     - Download Isaac Sim (Omniverse launcher)
     - System requirements (GPU, RAM, disk)
     - Launching Isaac Sim GUI
   - **3. Importing Humanoid Models** (FR-003)
     - URDF import process
     - Converting URDF to USD (Universal Scene Description)
     - Verifying model in scene
   - **4. Sensor Configuration** (FR-004)
     - Adding RGB cameras, depth sensors, LiDAR
     - Configuring sensor parameters (resolution, FOV, frame rate)
     - Visualizing sensor outputs
   - **5. Running Simulations** (FR-005)
     - Creating realistic environments (indoor, outdoor)
     - Physics settings (gravity, friction, collisions)
     - Playing simulation and observing robot behavior
   - **6. Synthetic Data Generation** (FR-006)
     - Exporting annotated images (bounding boxes, segmentation masks)
     - Dataset format and structure
     - Use case: training perception models
   - **7. Domain Randomization** (FR-007)
     - Lighting variations (time of day, intensity)
     - Texture randomization (object appearances)
     - Object placement randomization (clutter, obstacles)
   - **8. ROS 2 Integration** (FR-008)
     - Isaac Sim ROS 2 bridge
     - Publishing sensor data to ROS topics
     - Subscribing to command topics
     - Architecture diagram (Isaac Sim ↔ ROS 2)
   - **9. Performance Tips** (FR-009)
     - GPU utilization optimization
     - Cloud deployment options (AWS, GCP)
     - Headless mode for automation
5. **Summary and Next Steps**:
   - Recap: Isaac Sim enables safe, fast, photorealistic humanoid testing
   - Next chapter: Use Isaac ROS for real-time perception on simulated robots
   - Link to Chapter 2 (isaac-ros.md)

**Code Examples**:
- URDF import command (bash)
- Python script to randomize environment
- ROS 2 launch file for Isaac Sim bridge

**Spec Traceability**: FR-001 to FR-009 (spec.md:80-88)

---

#### File 3: `frontend_book/docs/module-3-isaac/02-isaac-ros.md`
**Purpose**: Chapter 2 - Teach Isaac ROS GPU-accelerated perception and VSLAM

**Content Sections** (estimated 2000-2500 words):
1. **Frontmatter**:
   - `sidebar_position: 2`
   - `slug: isaac-ros`
   - `title: "Isaac ROS: Hardware-Accelerated Perception"`
   - `sidebar_label: "Isaac ROS"`
   - `tags: [isaac-ros, vslam, perception, gpu-acceleration, visual-odometry]`
2. **Learning Objectives** (5-6 bullets mapping to spec FR-010 to FR-019)
3. **Prerequisites** (callout):
   - Chapter 1 (Isaac Sim fundamentals)
   - Module 1 Chapter 2 (ROS 2 publishers/subscribers)
   - NVIDIA GPU with CUDA support
4. **Main Content**:
   - **1. Introduction: Why Isaac ROS?** (FR-010)
     - Real-time perception challenges (30+ FPS for locomotion)
     - CPU vs GPU perception (latency comparison)
     - Isaac ROS performance benefits (2-5x speedup)
   - **2. Installation** (FR-011)
     - Isaac ROS apt repository
     - CUDA/cuDNN dependencies
     - Verifying GPU access
   - **3. Visual SLAM Pipeline Setup** (FR-012)
     - Isaac ROS Visual SLAM package
     - Configuring stereo cameras or depth sensors
     - Launching VSLAM node
   - **4. VSLAM Concepts** (FR-013)
     - Feature extraction (ORB, FAST, SIFT)
     - Feature matching and tracking
     - Pose estimation (visual odometry)
     - Map building (keyframes, loop closure)
     - Diagram: VSLAM pipeline flowchart
   - **5. Subscribing to Perception Outputs** (FR-014)
     - Pose estimate topic (`/visual_slam/tracking/odometry`)
     - Point cloud topic (`/visual_slam/vis/point_cloud`)
     - Occupancy grid topic (`/visual_slam/vis/map`)
     - Python subscriber example
   - **6. Visualization with RViz2** (FR-015)
     - Launching RViz2 with Isaac ROS topics
     - Displaying camera images, feature tracks
     - 3D point cloud rendering
     - Robot trajectory visualization
   - **7. Parameter Tuning** (FR-016)
     - Feature detection thresholds
     - Matching criteria (min inliers, RANSAC)
     - Keyframe selection (distance, angle)
     - Environment-specific tuning (indoor vs outdoor)
   - **8. Troubleshooting and Failure Modes** (FR-017)
     - Feature-poor environments (blank walls)
     - Rapid motion (motion blur)
     - Dynamic scenes (moving people/objects)
     - Mitigation strategies
   - **9. Performance Comparison** (FR-018)
     - Table: Isaac ROS vs rtabmap vs ORB-SLAM3
     - Metrics: FPS, latency, CPU/GPU usage
     - When to use Isaac ROS vs alternatives
   - **10. Testing in Isaac Sim** (FR-019)
     - Running Isaac ROS pipeline with simulated sensors
     - Validating perception before hardware deployment
     - Sim-to-real transfer considerations
5. **Summary and Next Steps**:
   - Recap: Isaac ROS enables real-time perception for humanoid control
   - Next chapter: Use Nav2 to plan paths and navigate autonomously
   - Link to Chapter 3 (nav2-humanoids.md)

**Code Examples**:
- Isaac ROS launch file (Python launch script)
- RViz2 configuration file
- Python subscriber to pose estimates

**Spec Traceability**: FR-010 to FR-019 (spec.md:92-101)

---

#### File 4: `frontend_book/docs/module-3-isaac/03-nav2-humanoids.md`
**Purpose**: Chapter 3 - Teach Nav2 path planning and navigation for bipedal robots

**Content Sections** (estimated 2500-3000 words):
1. **Frontmatter**:
   - `sidebar_position: 3`
   - `slug: nav2-humanoids`
   - `title: "Nav2 for Humanoids: Bipedal Navigation"`
   - `sidebar_label: "Nav2 for Humanoids"`
   - `tags: [nav2, navigation, path-planning, bipedal-robots, obstacle-avoidance]`
2. **Learning Objectives** (6-7 bullets mapping to spec FR-020 to FR-031)
3. **Prerequisites** (callout):
   - Chapter 1 (Isaac Sim)
   - Chapter 2 (Isaac ROS perception)
   - Module 1 Chapter 2 (ROS 2 actions)
4. **Main Content**:
   - **1. Introduction: Bipedal Navigation Challenges** (FR-020)
     - Wheeled vs bipedal robots (stability, constraints)
     - Step height, foothold planning, balance
     - Why Nav2 + custom plugins for humanoids
   - **2. Installation and Setup** (FR-021)
     - Nav2 package installation (apt or build from source)
     - Verifying Nav2 components
     - Nav2 architecture overview
   - **3. Costmap Configuration** (FR-022)
     - Global costmap (static map from VSLAM)
     - Local costmap (dynamic obstacles)
     - Integrating Isaac ROS perception data (point clouds)
     - Costmap layers (static, obstacle, inflation)
   - **4. Behavior Trees and Plugins** (FR-023)
     - Nav2 BT (Behavior Tree) XML structure
     - Key plugins: planner, controller, recovery
     - How BTs coordinate navigation behaviors
   - **5. Bipedal-Specific Constraints** (FR-024)
     - Robot footprint (humanoid dimensions)
     - Step height limits (stairs, curbs)
     - Foothold stability (surface friction, slope)
     - Balance considerations (center of mass)
     - Foot placement planning (future work reference)
   - **6. Sending Navigation Goals** (FR-025)
     - Nav2 action client (Python example)
     - Goal pose format (position + orientation)
     - Monitoring navigation progress (feedback)
     - Canceling navigation
   - **7. Path Planning Algorithms** (FR-026)
     - A* (grid-based, optimal)
     - Dijkstra (shortest path, complete)
     - Smac Planner (hybrid A*, kinematic constraints)
     - When to use each for humanoids
   - **8. Local Trajectory Planning** (FR-027)
     - DWA (Dynamic Window Approach)
     - TEB (Timed Elastic Band)
     - Local planner tuning (velocity, acceleration limits)
   - **9. Dynamic Obstacle Avoidance** (FR-028)
     - Costmap updates from real-time sensor data
     - Replanning triggers (blocked path)
     - Example: navigating through crowd
   - **10. Recovery Behaviors** (FR-029)
     - Rotate in place (stuck facing obstacle)
     - Back up (deadlock escape)
     - Clear costmap (spurious obstacles)
     - When recovery behaviors trigger
   - **11. Full Navigation Workflow** (FR-030)
     - End-to-end example in Isaac Sim:
       1. Set goal pose (10m away)
       2. Nav2 plans global path
       3. Local planner executes trajectory
       4. Dynamic obstacle appears → replan
       5. Reach goal successfully
     - Visualization in RViz2 (planned path, local costmap)
   - **12. Safety for Physical Hardware** (FR-031)
     - Simulation-first workflow (never skip testing)
     - Safety zones (virtual barriers)
     - Emergency stop integration
     - Gradual deployment (low speed → full speed)
5. **Summary and Module Completion**:
   - Recap: Nav2 enables autonomous navigation for humanoid robots
   - Full Isaac stack: Simulation → Perception → Navigation
   - What's next: Module 4 (if exists) or apply skills to real projects
   - Link back to Module 3 overview

**Code Examples**:
- Nav2 parameter YAML file (costmap, planner, controller configs)
- Python action client for navigation goals
- RViz2 Nav2 visualization setup

**Spec Traceability**: FR-020 to FR-031 (spec.md:105-116)

---

#### File 5: `frontend_book/sidebars.ts` (modification)
**Purpose**: Add Module 3 to sidebar navigation

**Change Type**: Addition (insert new category after Module 2)

**Exact Code Addition** (after line 45 in sidebars.ts):
```typescript
{
  type: 'category',
  label: 'Module 3: The AI-Robot Brain',
  link: {
    type: 'doc',
    id: 'module-3-isaac/index',
  },
  items: [
    'module-3-isaac/isaac-sim',
    'module-3-isaac/isaac-ros',
    'module-3-isaac/nav2-humanoids',
  ],
},
```

**Validation**:
- Run `npm run build` in `frontend_book/` directory
- Verify no broken links or missing docs warnings
- Manually test navigation: click Module 3 → chapters → back to module

**Spec Traceability**: Addresses user request "Add Module-3 to the Docusaurus documentation structure"

---

### Non-Functional Requirements

#### NFR-1: Page Load Performance
**Target**: <2 seconds for any module page (measured via Lighthouse)

**Justification**: Static site generation (Docusaurus) + Markdown = inherently fast. No dynamic content or heavy images in this phase.

**Measurement**: Run Lighthouse audit on deployed GitHub Pages site

---

#### NFR-2: SEO and Discoverability
**Target**: Each chapter page scores 90+ on Lighthouse SEO audit

**Implementation**:
- Descriptive `title` and `description` in frontmatter
- Semantic HTML (Docusaurus generates proper heading hierarchy)
- `tags` for content categorization
- Internal linking (chapter navigation)

**Validation**: Lighthouse SEO audit after deployment

---

#### NFR-3: Mobile Responsiveness
**Target**: Readable on mobile devices (320px width minimum)

**Justification**: Docusaurus theme is mobile-responsive by default. No custom CSS required.

**Validation**: Manual testing on mobile viewport in browser dev tools

---

#### NFR-4: Accessibility (WCAG 2.1 Level AA)
**Target**: No critical accessibility violations

**Implementation**:
- Semantic Markdown (proper heading levels)
- Alt text for images (if added in future)
- Color contrast (default Docusaurus theme compliant)

**Validation**: Lighthouse accessibility audit

---

### Quickstart: Writing Module 3 Content

**Prerequisites**:
1. Docusaurus development environment running (`npm start` in `frontend_book/`)
2. This plan document (plan.md) open for reference
3. Spec document (spec.md) open for FR traceability

**Step-by-Step Workflow**:

1. **Create Module 3 Directory**:
   ```bash
   mkdir -p frontend_book/docs/module-3-isaac
   ```

2. **Create `index.md`** (Module 3 overview):
   - Copy frontmatter from `module-2-simulation/index.md`
   - Replace title, description, tags
   - Write content following "File 1" breakdown (see Phase 1 above)
   - Save and verify in browser (`http://localhost:3000/docs/module-3-isaac`)

3. **Create `01-isaac-sim.md`** (Chapter 1):
   - Copy frontmatter from `module-1-ros2/01-ros2-fundamentals.md`
   - Update for Isaac Sim (title, slug, tags)
   - Write content following "File 2" breakdown
   - Reference spec.md FR-001 to FR-009 for learning objectives
   - Save and verify navigation works

4. **Create `02-isaac-ros.md`** (Chapter 2):
   - Copy frontmatter from `01-isaac-sim.md`, update for Isaac ROS
   - Write content following "File 3" breakdown
   - Reference spec.md FR-010 to FR-019
   - Add code examples (Python, bash, YAML)

5. **Create `03-nav2-humanoids.md`** (Chapter 3):
   - Copy frontmatter from `02-isaac-ros.md`, update for Nav2
   - Write content following "File 4" breakdown
   - Reference spec.md FR-020 to FR-031
   - Include end-to-end navigation workflow

6. **Update `sidebars.ts`**:
   - Open `frontend_book/sidebars.ts`
   - Insert Module 3 category after Module 2 (see "File 5" breakdown)
   - Save and verify sidebar shows Module 3 with 3 chapters

7. **Validate Build**:
   ```bash
   cd frontend_book
   npm run build
   ```
   - Check for errors or warnings
   - Fix any broken links

8. **Manual Testing**:
   - Start dev server: `npm start`
   - Navigate through all Module 3 pages
   - Test chapter navigation links
   - Verify sidebar highlighting

**Estimated Time**: 6-8 hours for initial content draft (2-3 hours per chapter + 1 hour for index + 1 hour for testing)

---

### Testing Strategy

#### Unit-Level Testing
**N/A** - No code logic to unit test (pure documentation)

---

#### Integration Testing

**Test 1: Docusaurus Build**
```bash
cd frontend_book
npm run build
```
**Expected**: Zero errors, zero warnings, successful build output

**Pass Criteria**: Build completes without throwing exceptions

---

**Test 2: Broken Link Detection**
```bash
npm run build
# Docusaurus automatically detects broken internal links
```
**Expected**: No "Broken link" warnings in build output

**Pass Criteria**: All internal links (between chapters, to other modules) resolve correctly

---

**Test 3: Frontmatter Validation**
**Manual Check**:
- Each file has required fields: `sidebar_position`, `title`, `description`
- Chapter files include `slug`, `sidebar_label`, `tags`
- Sidebar positions are unique within module (0, 1, 2, 3)

**Pass Criteria**: Frontmatter conforms to Docusaurus schema, no warnings

---

#### Manual Testing Checklist

**Test 4: Navigation Flow**
1. Load `http://localhost:3000/docs/module-3-isaac`
2. Verify Module 3 appears in sidebar
3. Click Module 3 → should load index.md
4. Click Chapter 1 (Isaac Sim) → should load 01-isaac-sim.md
5. Click "Next" link → should go to Chapter 2
6. Repeat for all chapters
7. Click "Back to Module 3 Overview" → should return to index.md

**Pass Criteria**: All navigation links work, no 404 errors

---

**Test 5: Content Rendering**
1. Open each chapter page
2. Verify headings render correctly (# = h1, ## = h2, etc.)
3. Verify code blocks have syntax highlighting
4. Verify callouts render (`:::info`, `:::tip`, `:::warning`)
5. Verify tables render with proper formatting

**Pass Criteria**: All Markdown elements render as expected

---

**Test 6: Mobile Responsiveness**
1. Open DevTools → Toggle device toolbar
2. Set viewport to 320px width (iPhone SE)
3. Navigate through all Module 3 pages
4. Verify text is readable, no horizontal scrolling
5. Verify sidebar collapses to hamburger menu

**Pass Criteria**: Pages are usable on small screens

---

**Test 7: Accessibility Audit**
1. Open Lighthouse in Chrome DevTools
2. Run audit on `http://localhost:3000/docs/module-3-isaac`
3. Verify Accessibility score ≥90
4. Check for critical violations (color contrast, heading hierarchy)

**Pass Criteria**: No critical accessibility issues

---

#### Acceptance Testing

**Acceptance Criterion 1**: Module 3 appears in Docusaurus sidebar
**Test**: Load docs homepage, verify "Module 3: The AI-Robot Brain" appears in left sidebar after Module 2
**Status**: [PENDING - to be verified during implementation]

---

**Acceptance Criterion 2**: All 3 chapters are accessible and render correctly
**Test**: Navigate to each chapter (isaac-sim, isaac-ros, nav2-humanoids), verify content displays without errors
**Status**: [PENDING]

---

**Acceptance Criterion 3**: Learning objectives map to spec requirements
**Test**: Open spec.md FR-001 to FR-031, verify each requirement is addressed in corresponding chapter
**Status**: [PENDING - content review required]

---

**Acceptance Criterion 4**: Build succeeds without errors
**Test**: Run `npm run build`, verify exit code 0
**Status**: [PENDING]

---

### Deployment Plan

**Phase**: Static Site Generation + GitHub Pages Deployment

**Steps**:
1. **Local Build**:
   ```bash
   cd frontend_book
   npm run build
   ```
   Output: `frontend_book/build/` directory with static HTML/CSS/JS

2. **Commit Changes**:
   ```bash
   git add frontend_book/docs/module-3-isaac/
   git add frontend_book/sidebars.ts
   git commit -m "Add Module 3: The AI-Robot Brain (Isaac) with 3 chapters

   - Add module-3-isaac directory with index + 3 chapters
   - Update sidebars.ts to include Module 3 navigation
   - Content covers Isaac Sim, Isaac ROS, Nav2 for humanoids
   - Closes specs/003-isaac-ai-brain/spec.md FR-001 to FR-031

   🤖 Generated with [Claude Code](https://claude.com/claude-code)

   Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>"
   ```

3. **Push to Remote**:
   ```bash
   git push origin 003-isaac-ai-brain
   ```

4. **Create Pull Request** (via `gh` CLI or GitHub UI):
   ```bash
   gh pr create --title "Add Module 3: The AI-Robot Brain (NVIDIA Isaac)" --body "$(cat <<'EOF'
   ## Summary
   Adds Module 3 to the Docusaurus documentation structure with 4 new Markdown files covering NVIDIA Isaac ecosystem for humanoid robotics.

   ## Changes
   - ✅ Created `frontend_book/docs/module-3-isaac/` directory
   - ✅ Added `index.md` (Module 3 overview with prerequisites, learning path)
   - ✅ Added `01-isaac-sim.md` (Chapter 1: Photorealistic simulation and synthetic data)
   - ✅ Added `02-isaac-ros.md` (Chapter 2: GPU-accelerated VSLAM and perception)
   - ✅ Added `03-nav2-humanoids.md` (Chapter 3: Bipedal path planning and navigation)
   - ✅ Updated `sidebars.ts` to include Module 3 navigation

   ## Spec Alignment
   - Addresses spec.md FR-001 to FR-031 (all functional requirements)
   - Maps to User Stories P1 (Isaac Sim), P2 (Isaac ROS), P3 (Nav2)
   - Satisfies Success Criteria SC-001 to SC-010

   ## Testing
   - ✅ `npm run build` succeeds with zero errors
   - ✅ All internal links validated (no broken references)
   - ✅ Manual navigation testing (sidebar, chapter links)
   - ✅ Mobile responsiveness verified (320px viewport)

   ## Preview
   Run locally: `cd frontend_book && npm start`
   Navigate to: http://localhost:3000/docs/module-3-isaac

   🤖 Generated with [Claude Code](https://claude.com/claude-code)
   EOF
   )"
   ```

5. **GitHub Pages Deployment** (automatic on merge to main):
   - Docusaurus build runs via GitHub Actions (if configured)
   - Static site deploys to `https://your-username.github.io/frontend_book/docs/module-3-isaac`

**Rollback Plan**:
- If build fails: revert commit with `git revert <commit-sha>`
- If content issues found: create hotfix branch, update content, merge via fast-track PR

---

## Phase 2: Implementation Tasks (Preview)

> **Note**: Detailed tasks will be generated via `/sp.tasks` command (not part of `/sp.plan`)

**High-Level Task Categories** (for `/sp.tasks` to expand):

1. **Setup Phase**:
   - Create `module-3-isaac` directory
   - Initialize `index.md` with frontmatter boilerplate

2. **Content Creation Phase**:
   - Write `index.md` (Module 3 overview)
   - Write `01-isaac-sim.md` (Chapter 1)
   - Write `02-isaac-ros.md` (Chapter 2)
   - Write `03-nav2-humanoids.md` (Chapter 3)

3. **Configuration Phase**:
   - Update `sidebars.ts` with Module 3 entry

4. **Validation Phase**:
   - Run `npm run build` and fix errors
   - Manual navigation testing
   - Accessibility audit

5. **Deployment Phase**:
   - Commit changes
   - Create pull request
   - Merge to main

**Estimated Total Effort**: 8-10 hours (content writing dominates)

---

## Risks and Mitigation

### Risk 1: Sidebar Configuration Error
**Likelihood**: Medium | **Impact**: High (breaks navigation)

**Scenario**: Typo in `sidebars.ts` (wrong path, missing comma) causes build failure

**Mitigation**:
- Copy-paste exact structure from Module 2 configuration
- Validate with `npm run build` immediately after change
- Use TypeScript type checking (sidebars.ts is typed)

**Contingency**: Revert `sidebars.ts` change, fix error, rebuild

---

### Risk 2: Broken Internal Links
**Likelihood**: Medium | **Impact**: Medium (poor UX, SEO penalty)

**Scenario**: Chapter links reference wrong slugs (e.g., `01-isaac-sim` instead of `isaac-sim`)

**Mitigation**:
- Use relative links: `./isaac-sim.md` (Docusaurus resolves automatically)
- Run `npm run build` to detect broken links (Docusaurus warns)
- Manual testing of all navigation links

**Contingency**: Fix links in affected files, rebuild, redeploy

---

### Risk 3: Frontmatter Schema Mismatch
**Likelihood**: Low | **Impact**: Medium (pages don't render correctly)

**Scenario**: Missing required frontmatter fields (e.g., no `sidebar_position`)

**Mitigation**:
- Use frontmatter checklist (Phase 1 → Integration Testing → Test 3)
- Copy-paste from working module files
- Docusaurus build fails with clear error messages if frontmatter invalid

**Contingency**: Add missing fields, rebuild

---

### Risk 4: Content Scope Creep
**Likelihood**: Medium | **Impact**: Low (delays delivery)

**Scenario**: Spend excessive time on advanced topics, tutorial depth, code examples

**Mitigation**:
- Follow word count estimates (Phase 1 file breakdowns: 800-1000 for index, 2000-2500 per chapter)
- Timebox content creation (2-3 hours per chapter max for first draft)
- Mark advanced topics as "Future Work" if too complex

**Contingency**: Publish MVP content (core learning objectives only), iterate later

---

### Risk 5: Docusaurus Version Incompatibility
**Likelihood**: Low | **Impact**: High (build breaks)

**Scenario**: Future Docusaurus v4 flag (`future.v4: true`) causes unexpected behavior

**Mitigation**:
- Use stable Docusaurus patterns (observed in Module 1 & Module 2)
- Avoid experimental features
- Test build locally before pushing

**Contingency**: Revert `future.v4` flag in `docusaurus.config.ts` if issues arise

---

## Open Questions

### Q1: Should we include video embeds (YouTube tutorials)?
**Status**: DEFERRED

**Context**: Module 1 and Module 2 have no video embeds (text-only)

**Decision**: Keep Module 3 consistent (text + images + code blocks). Videos can be added later via content updates.

---

### Q2: Should we add interactive code playgrounds (e.g., embedded terminals)?
**Status**: DEFERRED

**Context**: Docusaurus supports plugins for interactive elements, but existing modules don't use them.

**Decision**: Follow existing pattern (static code blocks with copy-paste). Interactive elements are out of scope for this feature (can be added in future enhancement).

---

### Q3: How detailed should Isaac Sim installation instructions be?
**Status**: RESOLVED

**Decision**: Provide high-level steps with links to official NVIDIA docs (avoid duplicating vendor documentation that may change). Focus on "what you need to know" not "click-by-click tutorial".

**Rationale**: Spec FR-002 says "Chapter MUST demonstrate installing Isaac Sim" (not "provide complete installation guide"). High-level + external links satisfy requirement.

---

### Q4: Should we create a Module 3 GitHub Issues label for future content updates?
**Status**: OUT OF SCOPE

**Context**: Not required for initial implementation. Can be added later for content maintenance workflow.

**Decision**: Skip for now. Focus on core content delivery.

---

## Success Criteria (from Spec)

**Alignment Check**: How does this plan satisfy spec.md Success Criteria SC-001 to SC-010?

| Spec SC | Description | Plan Alignment |
|---------|-------------|----------------|
| SC-001 | Developers can set up Isaac Sim in 2 hours | Chapter 1 Section 2 (Installation) provides step-by-step guide |
| SC-002 | Export 1000 annotated images | Chapter 1 Section 6 (Synthetic Data) explains export process |
| SC-003 | Isaac ROS VSLAM at 15 FPS | Chapter 2 Section 9 (Performance) includes benchmark table |
| SC-004 | Visualize perception in RViz2 | Chapter 2 Section 6 (Visualization) provides RViz2 setup |
| SC-005 | Navigate 10m avoiding 3 obstacles | Chapter 3 Section 11 (Full Workflow) demonstrates end-to-end example |
| SC-006 | 90% complete hands-on examples | All chapters include practical examples (Isaac Sim-based, no hardware) |
| SC-007 | Articulate Isaac ROS benefits | Chapter 2 Section 1 (Why Isaac ROS?) explains 2-5x speedup |
| SC-008 | Configure 5 bipedal Nav2 parameters | Chapter 3 Section 5 (Bipedal Constraints) lists parameters |
| SC-009 | 80% perception accuracy | Chapter 1 Section 7 (Domain Randomization) explains how to achieve generalization |
| SC-010 | Complete workflow in 4 hours | All 3 chapters designed for ~4 hour total reading time (1-1.5 hours each) |

**Verdict**: ✅ All 10 success criteria addressed in plan

---

## Conclusion

This plan provides a comprehensive, actionable blueprint for adding Module 3 to the Docusaurus documentation structure. By following the established patterns from Module 1 and Module 2, we ensure consistency, maintainability, and a smooth user experience.

**Next Steps**:
1. Review this plan (validate alignment with spec.md)
2. Run `/sp.tasks` to generate detailed, testable task list
3. Execute tasks via `/sp.implement`
4. Deploy to GitHub Pages

**Estimated Total Effort**: 8-10 hours (content writing dominates)

**Key Deliverables**:
- 4 new Markdown files (index + 3 chapters)
- 1 modified TypeScript file (sidebars.ts)
- Validated Docusaurus build
- Deployed documentation on GitHub Pages

---

*Plan generated via `/sp.plan` command on 2025-12-25*
*Spec: specs/003-isaac-ai-brain/spec.md*
*Branch: 003-isaac-ai-brain*
