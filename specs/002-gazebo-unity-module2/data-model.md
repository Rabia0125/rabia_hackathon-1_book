# Data Model: Module 2 Content Structure

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-24

## Content Entity Model

This document defines the structure and relationships of content entities for Module 2, ensuring consistency across all three chapters.

---

## Core Entities

### Module

**Attributes**:
- `id`: "module-2-simulation"
- `title`: "Module 2: The Digital Twin (Gazebo & Unity)"
- `sidebar_position`: 2 (follows Module 1)
- `description`: "Physics-based simulation and digital twins for humanoid robots"
- `duration`: "~4 hours"
- `prerequisites`: ["Module 1 completion", "ROS 2 knowledge", "URDF understanding"]
- `chapters`: Array of Chapter entities

**Relationships**:
- HAS-MANY chapters (3 chapters)
- FOLLOWS module-1-ros2

---

### Chapter

**Attributes**:
- `id`: String (e.g., "01-gazebo-simulation")
- `title`: String (e.g., "Chapter 1: Gazebo Simulation Basics")
- `sidebar_label`: String (short title for navigation)
- `sidebar_position`: Integer (1, 2, or 3)
- `description`: String (2-sentence summary)
- `tags`: Array of strings (e.g., ["gazebo", "physics", "simulation"])
- `learning_objectives`: Array of strings (5-6 objectives)
- `prerequisites`: Array of strings
- `estimated_time`: String (e.g., "90 minutes")

**Content Structure**:
- `sections`: Array of Section entities (ordered)
- `code_examples`: Array of CodeExample entities
- `diagrams`: Array of Diagram entities
- `summary`: Summary entity
- `self_assessment`: Array of Question entities

**Relationships**:
- BELONGS-TO module
- HAS-MANY sections (6-8 sections per chapter)
- HAS-MANY code_examples (5-7 per chapter)
- HAS-MANY diagrams (2-3 per chapter)
- HAS-ONE summary
- HAS-MANY self_assessment questions (4-5 per chapter)

---

### Section

**Attributes**:
- `heading_level`: Integer (2 for ## or 3 for ###)
- `title`: String
- `content`: Markdown text
- `order`: Integer (position within chapter)

**Types of Sections**:
1. **Introduction Section**: Sets context, motivation
2. **Conceptual Section**: Explains theory, architecture
3. **Tutorial Section**: Step-by-step instructions
4. **Code Example Section**: Demonstrates implementation
5. **Comparison Section**: Gazebo vs Unity trade-offs
6. **Best Practices Section**: Tips, warnings, optimization

**Validation Rules**:
- First section must be introduction
- Last section must be summary
- Each chapter must have at least one tutorial section
- Each chapter must have at least one code example section

---

### CodeExample

**Attributes**:
- `language`: String (e.g., "python", "xml", "bash", "csharp")
- `title`: String (descriptive title)
- `code`: String (complete, runnable code)
- `explanation`: Markdown text (line-by-line walkthrough)
- `expected_output`: String (what running the code produces)
- `filename_suggestion`: String (e.g., "simple_world.world")

**Types by Chapter**:

**Chapter 1 (Gazebo)**:
- World files (XML)
- Launch files (XML)
- ROS 2 spawn commands (bash)
- Python control scripts

**Chapter 2 (Unity)**:
- Unity C# scripts for ROS 2 communication
- ROS 2 launch files for Unity bridge
- Python scripts for testing Unity simulation

**Chapter 3 (Sensors)**:
- URDF sensor definitions (XML)
- Gazebo sensor plugins (XML)
- Unity sensor scripts (C#)
- ROS 2 subscriber nodes (Python)

**Validation Rules**:
- All code must be syntax-highlighted with correct language tag
- All code must include inline comments
- All code must be complete (no "..." placeholders)
- Python code must follow PEP 8 style
- XML must be properly indented

---

### Diagram

**Attributes**:
- `type`: String ("mermaid" or "image")
- `title`: String
- `caption`: String (explanation text below diagram)
- `content`: String (Mermaid code or image path)
- `alt_text`: String (for accessibility)

**Diagram Types by Chapter**:

**Chapter 1 (Gazebo)**:
- Gazebo architecture diagram (system components)
- Physics engine comparison flowchart
- World file structure diagram

**Chapter 2 (Unity)**:
- Unity Robotics Hub architecture
- ROS 2 ↔ Unity communication flow
- Articulation Body joint hierarchy

**Chapter 3 (Sensors)**:
- Sensor data pipeline (sensor → ROS 2 → processing)
- LiDAR scan visualization
- Sensor placement on humanoid robot

**Preferred Format**: Mermaid diagrams (inline) for architectural/flow diagrams; PNG only if complex 3D visualization needed

---

### Summary

**Attributes**:
- `key_takeaways`: Array of strings (5-7 bullet points)
- `next_chapter_preview`: String (1-2 sentences linking to next chapter)
- `additional_resources`: Array of Link entities

**Validation Rules**:
- Key takeaways must map to learning objectives
- Each takeaway must be actionable/concrete
- Next chapter preview must exist for chapters 1-2

---

### Question (Self-Assessment)

**Attributes**:
- `type`: String ("multiple-choice", "true-false", "short-answer")
- `question_text`: String
- `options`: Array of strings (for multiple-choice)
- `correct_answer`: String or integer (index)
- `explanation`: String (why this is the correct answer)
- `difficulty`: String ("easy", "medium", "hard")

**Distribution per Chapter**:
- 2 easy questions (recall/recognition)
- 2 medium questions (application/analysis)
- 1 hard question (synthesis/comparison)

**Topics Coverage**:
- Each learning objective must have at least 1 question
- Questions must test practical understanding, not memorization

---

## Cross-Cutting Entities

### Link

**Attributes**:
- `text`: String (link label)
- `url`: String (external URL or internal path)
- `type`: String ("internal", "external", "reference")

**Types**:
- **Internal**: Links to other chapters/modules (e.g., "/docs/module-1-ros2/01-ros2-fundamentals")
- **External**: Official documentation (e.g., Gazebo docs, Unity docs)
- **Reference**: Code repositories, tutorials, papers

---

### CalloutBox

**Attributes**:
- `type`: String ("info", "tip", "warning", "danger", "note")
- `title`: String (optional)
- `content`: Markdown text

**Usage Patterns**:
- **info**: Prerequisites, definitions
- **tip**: Best practices, shortcuts, pro tips
- **warning**: Common mistakes, performance issues
- **danger**: Critical errors, system requirements
- **note**: Side information, historical context

**Validation Rules**:
- Each chapter must have a "Prerequisites" callout at the beginning
- System requirements callout must appear in Chapter 1
- "Gazebo vs Unity" comparison callout must appear in Chapter 2

---

## Relationships Diagram

```
Module
  └─ HAS-MANY Chapters (3)
       ├─ HAS-MANY Sections (6-8)
       │    └─ CONTAINS CodeExamples, Diagrams, CalloutBoxes
       ├─ HAS-MANY CodeExamples (5-7)
       ├─ HAS-MANY Diagrams (2-3)
       ├─ HAS-ONE Summary
       └─ HAS-MANY Questions (4-5)
```

---

## Validation Rules Summary

### Module Level
- Must have exactly 3 chapters
- Chapters must be numbered sequentially
- Total word count: 24,000-30,000 words
- Total code examples: 15-20
- Total diagrams: 6-9

### Chapter Level
- Must have frontmatter with all required fields
- Must have Learning Objectives section
- Must have Prerequisites callout
- Must have Summary section
- Must have Self-Assessment section
- Sections must follow logical order
- Code examples must be runnable
- All internal links must be valid

### Content Quality
- No orphaned placeholders (e.g., "[TODO]", "FIXME")
- No broken links
- All code blocks must have language tags
- All diagrams must have captions
- All callouts must have proper type
- Markdown must be valid (linting pass)

---

## Content Consistency Rules

### Terminology
- Use "Gazebo" not "Gazebo Simulator" or "Gazebo Classic"
- Use "Unity" not "Unity3D" or "Unity Engine"
- Use "ROS 2" not "ROS2" or "ROS 2.0"
- Use "URDF" not "urdf" or "Urdf"
- Use "digital twin" not "Digital Twin" (unless starting sentence)

### Code Style
- Python: PEP 8, type hints optional
- XML: 2-space indentation
- C#: Unity conventions, PascalCase for public methods
- Bash: Use `#!/usr/bin/env bash` shebang

### File Naming
- Markdown files: `##-lowercase-with-hyphens.md`
- Images: `descriptive-name.png` or `descriptive-name.svg`
- Mermaid diagrams: Inline in markdown, no separate files

---

This data model ensures structural consistency across all Module 2 content while maintaining alignment with Module 1's established patterns.
