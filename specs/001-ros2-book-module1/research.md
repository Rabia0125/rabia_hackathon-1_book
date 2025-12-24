# Research: Module 1 - The Robotic Nervous System (ROS 2)

**Phase**: 0 (Outline & Research)
**Date**: 2025-12-21
**Status**: Complete

## Research Topics

### 1. Docusaurus 3.x Documentation Structure

**Decision**: Use `docs/` folder with module subfolders and `_category_.json` for sidebar organization.

**Rationale**:
- Docusaurus 3.x uses file-system based routing in `docs/`
- `_category_.json` provides declarative sidebar configuration per folder
- MDX support enables React components in Markdown (future enhancement)
- Frontmatter controls per-page metadata (title, sidebar_position, tags)

**Alternatives Considered**:
- Manual `sidebars.js` configuration: More control but harder to maintain as content grows
- Blog structure: Not suitable for sequential educational content
- Versioned docs: Overkill for initial release; can add later

**Best Practices Applied**:
- Use `sidebar_position` in frontmatter for chapter ordering
- Keep file names lowercase with hyphens (`01-ros2-fundamentals.md`)
- Module folders prefixed with number for visual ordering (`module-1-ros2/`)

### 2. ROS 2 Target Version

**Decision**: ROS 2 Humble Hawksbill (LTS, supported until 2027)

**Rationale**:
- Humble is the current LTS release with widest adoption
- Most tutorials and resources target Humble
- Iron Irwini (2023) has shorter support window
- Jazzy Jalisco (2024) too new for stable educational content

**Alternatives Considered**:
- Iron Irwini: Newer but less documentation available
- Multi-version support: Adds complexity without clear benefit for beginners

**Best Practices Applied**:
- Explicitly state ROS 2 version in chapter prerequisites
- Note any version-specific behaviors in callout boxes
- Use standard message types (`geometry_msgs`, `sensor_msgs`) for compatibility

### 3. Code Example Strategy

**Decision**: All code examples must be complete, copy-paste ready Python scripts.

**Rationale**:
- Target audience (AI students) expects runnable code, not snippets
- Complete examples reduce friction and debugging time
- Python is the primary language for AI/ML integration

**Alternatives Considered**:
- Interactive notebooks: Require additional setup (Jupyter + ROS bridge)
- Partial snippets with "fill in the blank": Frustrating for beginners
- C++ examples: Higher barrier to entry for AI audience

**Best Practices Applied**:
- Include all imports at the top of each example
- Add `#!/usr/bin/env python3` shebang for executable scripts
- Show expected output in comments or separate output blocks
- Use type hints for clarity (Python 3.9+ style)

### 4. Nervous System Metaphor Mapping

**Decision**: Use consistent biological analogies throughout Module 1.

**Mapping**:
| ROS 2 Concept | Biological Analog | Explanation |
|---------------|-------------------|-------------|
| Node | Neuron | Independent processing unit with inputs/outputs |
| Topic | Sensory nerve | Continuous data stream (e.g., vision, touch) |
| Message | Neural signal | Structured data packet traveling through system |
| Service | Reflex arc | Request-response for quick, synchronous actions |
| Action | Motor command | Long-running task with feedback (e.g., walking) |
| DDS | Nervous system backbone | Communication infrastructure enabling all signals |

**Rationale**:
- AI students are familiar with neural network concepts
- Biological metaphors make abstract middleware concrete
- Consistent terminology reinforces learning

**Alternatives Considered**:
- Computer network metaphor (nodes, messages, protocols): Less intuitive for target audience
- No metaphor (pure technical): Misses pedagogical opportunity

### 5. URDF Humanoid Example

**Decision**: Create a simplified 15-link humanoid model for Chapter 3.

**Structure**:
```
base_link (torso)
├── head_link
├── left_arm_link
│   └── left_forearm_link
│       └── left_hand_link
├── right_arm_link
│   └── right_forearm_link
│       └── right_hand_link
├── left_hip_link
│   └── left_thigh_link
│       └── left_shin_link
│           └── left_foot_link
└── right_hip_link
    └── right_thigh_link
        └── right_shin_link
            └── right_foot_link
```

**Rationale**:
- 15 links is enough to demonstrate kinematic chains without overwhelming
- Covers all major humanoid components (arms, legs, head, torso)
- Simple enough to annotate fully in a chapter
- Does not reference proprietary robot designs

**Alternatives Considered**:
- Full humanoid (30+ links): Too complex for introductory material
- Simple arm only: Misses the "humanoid" aspect of the course
- Real robot URDF (TurtleBot, etc.): Not humanoid; licensed concerns

### 6. Diagram Requirements

**Decision**: Create 3-4 diagrams per chapter using Mermaid or static images.

**Required Diagrams**:
1. **Chapter 1**: ROS 2 architecture overview (nodes, topics, services)
2. **Chapter 1**: Nervous system analogy visual
3. **Chapter 2**: Publisher/Subscriber data flow
4. **Chapter 3**: URDF link-joint hierarchy tree

**Rationale**:
- Visual learners need diagrams for abstract concepts
- Mermaid diagrams render in Docusaurus without external tools
- Static PNG fallbacks for complex visuals (architecture, anatomy)

**Best Practices Applied**:
- Use consistent color scheme across all diagrams
- Include alt text for accessibility
- Keep diagrams simple; annotate in surrounding text

## Research Gaps Resolved

All technical context items resolved. No NEEDS CLARIFICATION items remaining.

## Next Steps

Proceed to Phase 1:
1. `data-model.md` - Define chapter content structure
2. `contracts/` - Create detailed chapter outlines
3. `quickstart.md` - Local development setup instructions
