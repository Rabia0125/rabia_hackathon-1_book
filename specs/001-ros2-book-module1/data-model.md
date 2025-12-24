# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

**Phase**: 1 (Design & Contracts)
**Date**: 2025-12-21

## Content Entities

This feature produces **documentation content**, not application data. The "data model" describes the structure of content artifacts.

### Entity: Chapter

A single Docusaurus Markdown file representing one chapter of the book.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `title` | string | Yes | Chapter title (appears in sidebar and page) |
| `sidebar_position` | number | Yes | Order within module (1, 2, 3) |
| `sidebar_label` | string | No | Short label for sidebar (defaults to title) |
| `description` | string | Yes | SEO meta description |
| `tags` | string[] | No | Content tags for search/filtering |
| `learning_objectives` | string[] | Yes | 3-5 objectives listed at chapter start |
| `prerequisites` | string[] | Yes | Required prior knowledge |
| `sections` | Section[] | Yes | Main content sections |
| `code_examples` | CodeExample[] | Yes | Runnable code snippets |
| `diagrams` | Diagram[] | No | Visual aids |
| `callouts` | Callout[] | No | Tips, warnings, ROS 1 notes |
| `summary` | string[] | Yes | Key takeaways (bullet points) |

**Validation Rules**:
- `sidebar_position` must be unique within module
- `learning_objectives` must have 3-5 items
- `prerequisites` must reference prior chapters or external knowledge
- All `code_examples` must be complete and runnable

### Entity: Section

A major heading within a chapter.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `heading` | string | Yes | Section title (H2 level) |
| `content` | string | Yes | Markdown prose content |
| `subsections` | Subsection[] | No | Nested content (H3 level) |

### Entity: CodeExample

A complete, runnable code snippet.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `language` | string | Yes | Syntax highlighting hint (`python`, `xml`, `bash`) |
| `filename` | string | No | Suggested filename for saving |
| `code` | string | Yes | Complete source code |
| `explanation` | string | Yes | Line-by-line or block explanation |
| `expected_output` | string | No | What the code produces when run |
| `dependencies` | string[] | No | Required ROS 2 packages |

**Validation Rules**:
- `language` must be one of: `python`, `xml`, `bash`, `yaml`, `json`
- `code` must include all imports and be copy-paste ready
- Python code must include `#!/usr/bin/env python3` shebang

### Entity: Diagram

A visual representation of concepts.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | enum | Yes | `mermaid` or `image` |
| `source` | string | Yes | Mermaid code or image path |
| `alt_text` | string | Yes | Accessibility description |
| `caption` | string | No | Figure caption |

### Entity: Callout

A highlighted information box.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | enum | Yes | `tip`, `warning`, `info`, `note`, `ros1` |
| `title` | string | No | Callout heading |
| `content` | string | Yes | Callout body text |

**Docusaurus Syntax**:
```markdown
:::tip Title
Content here
:::
```

## Content Relationships

```
Module 1
├── Chapter 1: ROS 2 Fundamentals
│   ├── Section: What is ROS 2?
│   ├── Section: The Nervous System Metaphor
│   ├── Section: Nodes and Topics
│   ├── Section: Services and Actions
│   ├── Section: DDS and Real-Time Communication
│   └── Summary
├── Chapter 2: Python Agents to Robot Control
│   ├── Section: Setting Up rclpy
│   ├── Section: Creating Your First Node
│   ├── Section: Publishing Commands
│   ├── Section: Subscribing to Sensors
│   ├── Section: Calling Services
│   ├── Section: Building an AI Agent
│   └── Summary
└── Chapter 3: Humanoid Modeling with URDF
    ├── Section: What is URDF?
    ├── Section: Links and Joints
    ├── Section: Visual and Collision Properties
    ├── Section: Sensors in URDF
    ├── Section: Humanoid-Specific Design
    ├── Section: Complete Humanoid Example
    └── Summary
```

## File Naming Convention

```
docs/module-1-ros2/
├── _category_.json           # Module metadata
├── index.md                  # Module overview (sidebar_position: 0)
├── 01-ros2-fundamentals.md   # Chapter 1 (sidebar_position: 1)
├── 02-python-ros-control.md  # Chapter 2 (sidebar_position: 2)
└── 03-humanoid-urdf.md       # Chapter 3 (sidebar_position: 3)
```

## Frontmatter Template

```yaml
---
title: "Chapter Title"
sidebar_position: 1
sidebar_label: "Short Label"
description: "SEO description for this chapter"
tags:
  - ros2
  - robotics
  - python
---
```
