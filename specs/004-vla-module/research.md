# Research: Module 4 VLA Documentation Patterns

**Date**: 2025-12-25
**Purpose**: Document existing Docusaurus patterns from Modules 1-3 to ensure Module 4 consistency

## Research Question 1: Docusaurus Frontmatter Structure

### Findings from Module 3 Analysis

**Module Index Frontmatter** (`module-3-isaac/index.md`):
```yaml
---
sidebar_position: 0
title: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
description: "Learn high-fidelity simulation, perception, and navigation using NVIDIA Isaac for humanoid robots"
---
```

**Chapter Frontmatter** (`module-3-isaac/01-isaac-sim.md`):
```yaml
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
```

### Decision: Frontmatter Schema for Module 4

**Module Index**:
- `sidebar_position`: 0 (always 0 for module index)
- `title`: Full module title with parenthetical subtitle
- `description`: One-sentence summary for SEO

**Chapters**:
- `sidebar_position`: Integer 1-3 (determines order in sidebar)
- `slug`: Kebab-case identifier (used in URLs: `/docs/module-4-vla/voice-to-action`)
- `title`: Full chapter title with colon separator
- `sidebar_label`: Short label for sidebar (3-5 words)
- `description`: One-sentence description for SEO and previews
- `tags`: Array of 3-5 relevant keywords (lowercase, hyphenated)

### Rationale
- **Slug-based routing** allows clean URLs independent of filename
- **Tags** improve discoverability and enable tag-based navigation
- **sidebar_position** controls display order without relying on filename sorting
- **sidebar_label** keeps sidebar compact while full title appears on page

---

## Research Question 2: Content Structure and Depth

### Findings from Module 3 Chapter Analysis

**Standard Chapter Structure** (from `module-3-isaac/01-isaac-sim.md`):

1. **Frontmatter** (7-10 fields)
2. **Title** (H1, matches frontmatter title)
3. **Learning Objectives** (H2)
   - Numbered list of 5-10 specific outcomes
   - Action-oriented ("be able to...")
4. **Prerequisites** (H2)
   - Info admonition (`:::info Before You Begin`)
   - Bullet list with links to prior modules/chapters
   - Hardware/software requirements
5. **Main Content Sections** (H2, typically 8-10 sections)
   - Each section 100-150 lines
   - Subsections (H3) for detailed topics
   - Code examples (```python or ```bash)
   - Admonitions for tips/warnings (`:::tip`, `:::warning`)
   - Tables for comparisons
6. **Summary and Next Steps** (H2)
   - "What You Accomplished" with checkboxes
   - "Key Takeaways" (3-5 bullet points)
   - "What's Next?" preview of next chapter
7. **Chapter Navigation** (H3 with bold)
   - Previous chapter link
   - Next chapter link
   - Back to module index link
8. **Footer** (Italics)
   - "Chapter X of Module Y: [Module Title]"

### Content Depth Guidelines

**Module 3 Example Analysis**:
- **Total lines per chapter**: 850-1200 lines
- **Code examples**: 10-15 per chapter
- **Tables**: 5-8 comparison/specification tables
- **Admonitions**: 8-12 tips/warnings/info boxes
- **Subsection depth**: Max 3 levels (H1 > H2 > H3)

**Theory vs Practice Balance**:
- 40% conceptual explanation
- 40% hands-on code examples
- 20% troubleshooting/tips

### Decision: Module 4 Content Depth

**Chapter 1: Voice-to-Action** (estimated 900 lines):
- Whisper installation and setup (150 lines)
- Speech-to-text pipeline (200 lines)
- Intent recognition with NLP (200 lines)
- ROS 2 action mapping (250 lines)
- Testing and debugging (100 lines)

**Chapter 2: Cognitive Planning** (estimated 1000 lines):
- LLM integration (API vs local) (200 lines)
- Task decomposition algorithms (250 lines)
- Plan validation and constraints (200 lines)
- Dynamic replanning (200 lines)
- Safety and explainability (150 lines)

**Chapter 3: Capstone Project** (estimated 1100 lines):
- End-to-end architecture (200 lines)
- Isaac Sim integration testing (250 lines)
- Multimodal feedback (voice + visual) (200 lines)
- Benchmark task walkthrough (300 lines)
- Physical hardware deployment (150 lines)

---

## Research Question 3: Navigation Links and Cross-References

### Findings from Module 3

**Cross-Module Links** (absolute paths):
```markdown
[Module 1: The Robotic Nervous System](/docs/module-1-ros2/)
[Module 2: The Digital Twin](/docs/module-2-simulation/)
```

**Intra-Module Links** (absolute paths to slugs):
```markdown
[Chapter 1: Isaac Sim](/docs/module-3-isaac/isaac-sim)
[Chapter 2: Isaac ROS](/docs/module-3-isaac/isaac-ros)
```

**Chapter Navigation Format** (at end of each chapter):
```markdown
**Chapter Navigation**:

- ← Previous: [Module 3 Overview](/docs/module-3-isaac/)
- → Next: [Chapter 2: Isaac ROS](/docs/module-3-isaac/isaac-ros)
- ↑ Back to Module 3: [Module 3 Index](/docs/module-3-isaac/)
```

### Decision: Module 4 Navigation Pattern

**Cross-Module Prerequisites**:
- Link to Module 1: `/docs/module-1-ros2/` (ROS 2 fundamentals)
- Link to Module 2: `/docs/module-2-simulation/` (simulation concepts)
- Link to Module 3: `/docs/module-3-isaac/` (perception/navigation)
- Link to specific chapters: `/docs/module-1-ros2/02-python-ros-control` (slug-based)

**Module 4 Internal Navigation**:
- Index: `/docs/module-4-vla/`
- Chapter 1: `/docs/module-4-vla/voice-to-action`
- Chapter 2: `/docs/module-4-vla/cognitive-planning`
- Chapter 3: `/docs/module-4-vla/capstone-project`

**Chapter Navigation Template**:
```markdown
**Chapter Navigation**:

- ← Previous: [Previous Chapter Title](/docs/module-4-vla/previous-slug)
- → Next: [Next Chapter Title](/docs/module-4-vla/next-slug)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)
```

### Rationale
- **Absolute paths** prevent broken links when files are moved
- **Slug-based URLs** are cleaner and more maintainable than filename-based
- **Consistent symbols** (←, →, ↑) match Modules 1-3 UX

---

## Research Question 4: Code Example Format and Testing

### Findings from Module 3

**Python Code Example Format**:
````markdown
```python
# Descriptive comment explaining what this code does
import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Node started')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
````

**Explanation Pattern** (after code):
```markdown
This code:
- Creates a ROS 2 node named `example_node`
- Logs a startup message
- Spins indefinitely waiting for callbacks

To run: `ros2 run your_package example_node`
```

**Bash Command Format**:
````markdown
```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-example-package

# Run the node
ros2 run example_package example_node
```
````

### Decision: Module 4 Code Example Standards

**Whisper Integration Example**:
- Show OpenAI Whisper setup and real-time transcription
- Include audio preprocessing (noise reduction, VAD)
- Demonstrate ROS 2 audio topic subscription

**LLM API Call Example**:
- Show API key management (environment variables)
- Include prompt engineering for robotics tasks
- Demonstrate error handling and retries

**Action Execution Example**:
- Map natural language to ROS 2 action goal
- Show action client setup and feedback handling
- Include timeout and failure recovery

**Code Example Checklist**:
- ✅ Includes imports and complete context
- ✅ Has descriptive comments explaining non-obvious logic
- ✅ Includes error handling
- ✅ Provides "How to run" instructions after code
- ✅ Specifies version requirements (e.g., `openai==1.0.0`)

---

## Summary of Research Findings

### Key Decisions

1. **Frontmatter**: Follow Module 3 pattern exactly (sidebar_position, slug, title, sidebar_label, description, tags)

2. **Content Structure**:
   - Learning Objectives → Prerequisites → 8-10 Main Sections → Summary → Navigation → Footer
   - 850-1200 lines per chapter
   - 10-15 code examples per chapter

3. **Navigation Links**:
   - Use absolute paths (`/docs/module-X/slug`)
   - Consistent symbols (←, →, ↑) for chapter navigation
   - Link to specific chapters by slug, not filename

4. **Code Examples**:
   - Complete, runnable code with imports
   - Descriptive comments
   - "How to run" instructions
   - Version pinning for external libraries

### Alternatives Considered

**Relative vs Absolute Links**:
- ❌ Relative paths (`./chapter-slug`) - brittle if files move
- ✅ Absolute paths (`/docs/module-4-vla/chapter-slug`) - robust and consistent

**Filename vs Slug Routing**:
- ❌ Filename-based (`01-voice-to-action.md` → `/01-voice-to-action`) - exposes implementation detail
- ✅ Slug-based (`slug: voice-to-action` → `/voice-to-action`) - clean URLs

**Content Depth**:
- ❌ Brief summaries (300-500 lines) - insufficient for hands-on learning
- ❌ Exhaustive references (2000+ lines) - overwhelming, hard to navigate
- ✅ Practical tutorials (800-1200 lines) - balances theory and practice

---

**Research Complete**: All patterns documented. Ready for Phase 1 (Design).
