# Data Model: Module 4 VLA Documentation

**Date**: 2025-12-25
**Purpose**: Define content entities and navigation structure for Module 4

## Content Entities

### Entity: Module

**Purpose**: Top-level organizational unit representing Module 4

**Attributes**:
- `title`: "Module 4: The AI-Robot Brain (VLA)"
- `description`: "Learn to integrate voice commands, LLM-based planning, and autonomous robot control for humanoid robots"
- `duration`: "~5 hours"
- `chapter_count`: 3
- `prerequisites`: ["Module 1", "Module 2", "Module 3"]
- `hardware_requirements`: ["NVIDIA GPU", "microphone", "speakers", "ROS 2-compatible robot"]

**Relationships**:
- Contains 3 Chapters (one-to-many)
- Depends on Module 1 (ROS 2 fundamentals)
- Depends on Module 2 (simulation concepts)
- Depends on Module 3 (Isaac perception/navigation)

**File Location**: `frontend_book/docs/module-4-vla/index.md`

---

### Entity: Chapter

**Purpose**: Individual learning unit within Module 4

**Attributes**:
- `sidebar_position`: Integer (1-3)
- `slug`: String (kebab-case, used in URLs)
- `title`: String (full chapter title)
- `sidebar_label`: String (short label for sidebar)
- `description`: String (one-sentence SEO description)
- `tags`: Array<String> (3-5 relevant keywords)
- `learning_objectives`: Array<String> (5-10 outcomes)
- `prerequisites`: Array<Object> (with title and link)
- `estimated_lines`: Integer (800-1200 per chapter)

**Relationships**:
- Belongs to Module 4 (many-to-one)
- Links to Previous Chapter (optional, null for Chapter 1)
- Links to Next Chapter (optional, null for Chapter 3)
- Contains multiple Code Examples (one-to-many)
- Contains multiple Content Sections (one-to-many)

**Instances**:
1. **Chapter 1**: Voice-to-Action
   - `slug`: "voice-to-action"
   - `sidebar_position`: 1
   - `file`: `01-voice-to-action.md`

2. **Chapter 2**: Cognitive Planning
   - `slug`: "cognitive-planning"
   - `sidebar_position`: 2
   - `file`: `02-cognitive-planning.md`

3. **Chapter 3**: Capstone Project
   - `slug`: "capstone-project"
   - `sidebar_position`: 3
   - `file`: `03-capstone-project.md`

---

### Entity: Code Example

**Purpose**: Runnable code demonstrating concepts

**Attributes**:
- `language`: String ("python", "bash", "yaml", "json")
- `code_block`: String (multi-line code)
- `description`: String (what the code does)
- `explanation`: String (how it works)
- `how_to_run`: String (execution instructions)
- `dependencies`: Array<String> (required packages/tools)
- `expected_output`: String (optional, what user should see)

**Relationships**:
- Embedded within Chapter (many-to-one)
- May reference other Code Examples (many-to-many, via comments)

**Example Types**:
1. **Whisper Integration**:
   - Language: Python
   - Shows: OpenAI Whisper setup, audio capture, real-time transcription
   - Dependencies: `openai-whisper`, `rclpy`, `audio_common`

2. **LLM API Call**:
   - Language: Python
   - Shows: API key management, prompt engineering, response parsing
   - Dependencies: `openai` or `anthropic`, `langchain`

3. **ROS 2 Action Mapping**:
   - Language: Python
   - Shows: Action client setup, goal sending, feedback handling
   - Dependencies: `rclpy`, `action_msgs`, `nav2_msgs`

4. **Bash Commands**:
   - Language: Bash
   - Shows: Installation, environment setup, testing
   - Dependencies: `ros-humble-*`, `pip` packages

---

### Entity: Content Section

**Purpose**: Major subdivision of a chapter (H2 heading)

**Attributes**:
- `title`: String (section heading)
- `order`: Integer (position within chapter)
- `subsections`: Array<Object> (H3 headings)
- `estimated_lines`: Integer (100-150 per section)
- `code_examples`: Integer (1-2 per section)
- `admonitions`: Array<Object> (tips, warnings, info boxes)

**Relationships**:
- Belongs to Chapter (many-to-one)
- Contains Code Examples (one-to-many)
- Contains Subsections (one-to-many)

**Example Sections for Chapter 1**:
1. **Introduction: Why Voice Control for Robots?**
   - Motivation and use cases
   - No code examples

2. **OpenAI Whisper Installation**
   - Step-by-step setup
   - 1 bash code example (pip install)
   - 1 warning (GPU requirements)

3. **Speech-to-Text Pipeline**
   - Architecture diagram (conceptual)
   - 2 Python code examples (audio capture, transcription)
   - 1 tip (optimizing for real-time)

4. **Intent Recognition with NLP**
   - Natural language understanding
   - 2 Python code examples (keyword extraction, intent classification)
   - 1 info box (using spaCy or NLTK)

5. **Mapping Commands to ROS 2 Actions**
   - Action interface overview
   - 3 Python code examples (action client, goal creation, feedback handling)
   - 1 warning (action timeouts)

6. **Testing and Debugging**
   - Validation strategies
   - 2 bash/Python examples (unit tests, integration tests)
   - 1 tip (using ROS 2 bag for replay)

---

### Entity: Navigation Link

**Purpose**: User navigation between chapters and modules

**Attributes**:
- `label`: String ("← Previous", "→ Next", "↑ Back to Module 4")
- `target_url`: String (absolute path like `/docs/module-4-vla/slug`)
- `link_text`: String (readable title like "Chapter 1: Voice-to-Action")

**Relationships**:
- Links between Chapters (many-to-many)
- Links from Chapter to Module Index (many-to-one)

**Navigation Structure**:

```
Chapter 1 (Voice-to-Action)
  ← Previous: Module 4 Overview (/docs/module-4-vla/)
  → Next: Chapter 2: Cognitive Planning (/docs/module-4-vla/cognitive-planning)
  ↑ Back: Module 4 Index (/docs/module-4-vla/)

Chapter 2 (Cognitive Planning)
  ← Previous: Chapter 1: Voice-to-Action (/docs/module-4-vla/voice-to-action)
  → Next: Chapter 3: Capstone Project (/docs/module-4-vla/capstone-project)
  ↑ Back: Module 4 Index (/docs/module-4-vla/)

Chapter 3 (Capstone Project)
  ← Previous: Chapter 2: Cognitive Planning (/docs/module-4-vla/cognitive-planning)
  ↑ Back: Module 4 Index (/docs/module-4-vla/)
  (No "Next" - this is the final chapter)
```

---

### Entity: Admonition

**Purpose**: Highlighted callout boxes for tips, warnings, or info

**Attributes**:
- `type`: String ("info", "tip", "warning", "danger", "note")
- `title`: String (optional heading for the admonition)
- `content`: String (body text, can include markdown)

**Usage Patterns**:
- **:::info**: For prerequisites, context, or background information
- **:::tip**: For best practices, optimization suggestions, or pro tips
- **:::warning**: For gotchas, common mistakes, or important limitations
- **:::danger**: For critical safety concerns or data loss risks (rare)

**Example Admonitions for Module 4**:

```markdown
:::info Hardware Requirements
You'll need an NVIDIA GPU (GTX 1060 or better) to run Whisper and Isaac ROS in real-time.
:::

:::tip Optimizing Whisper for Robotics
Use the `base` model (instead of `large`) for faster inference at the cost of slightly lower accuracy. For robotics commands, this trade-off is acceptable.
:::

:::warning API Rate Limits
OpenAI GPT-4 has rate limits (3 requests/minute for free tier). For production, cache common plans or use a local LLM.
:::
```

---

## Navigation Structure Diagram

```
Module 4: The AI-Robot Brain (VLA) [index.md]
│
├─ Prerequisites
│  ├─ Module 1: ROS 2 Fundamentals → /docs/module-1-ros2/
│  ├─ Module 2: Simulation → /docs/module-2-simulation/
│  └─ Module 3: Isaac Perception/Nav → /docs/module-3-isaac/
│
├─ Chapter 1: Voice-to-Action [01-voice-to-action.md]
│  ├─ Learning Objectives (5-10 items)
│  ├─ Prerequisites
│  │  └─ Module 1 Ch 2: Python + ROS 2 → /docs/module-1-ros2/02-python-ros-control
│  ├─ Section 1: Introduction
│  ├─ Section 2: Whisper Installation
│  │  └─ Code: pip install openai-whisper
│  ├─ Section 3: Speech-to-Text Pipeline
│  │  ├─ Code: Audio capture
│  │  └─ Code: Real-time transcription
│  ├─ Section 4: Intent Recognition
│  │  ├─ Code: Keyword extraction
│  │  └─ Code: Intent classification
│  ├─ Section 5: ROS 2 Action Mapping
│  │  ├─ Code: Action client setup
│  │  ├─ Code: Goal creation
│  │  └─ Code: Feedback handling
│  ├─ Section 6: Testing and Debugging
│  │  └─ Code: Unit/integration tests
│  ├─ Summary
│  └─ Chapter Navigation
│     ├─ ← Previous: Module 4 Overview
│     ├─ → Next: Chapter 2
│     └─ ↑ Back: Module 4 Index
│
├─ Chapter 2: Cognitive Planning [02-cognitive-planning.md]
│  ├─ Learning Objectives
│  ├─ Prerequisites
│  │  ├─ Chapter 1: Voice-to-Action → /docs/module-4-vla/voice-to-action
│  │  └─ Module 3: Isaac ROS → /docs/module-3-isaac/isaac-ros
│  ├─ Section 1: Introduction to LLM Planning
│  ├─ Section 2: LLM Integration (API vs Local)
│  │  ├─ Code: OpenAI API setup
│  │  └─ Code: Local Llama setup
│  ├─ Section 3: Task Decomposition
│  │  ├─ Code: Prompt engineering for robotics
│  │  └─ Code: Parsing LLM output to action sequence
│  ├─ Section 4: Plan Validation
│  │  └─ Code: Constraint checking
│  ├─ Section 5: Dynamic Replanning
│  │  └─ Code: Failure recovery
│  ├─ Section 6: Safety Constraints
│  │  └─ Code: Safety rule enforcement
│  ├─ Summary
│  └─ Chapter Navigation
│     ├─ ← Previous: Chapter 1
│     ├─ → Next: Chapter 3
│     └─ ↑ Back: Module 4 Index
│
└─ Chapter 3: Capstone Project [03-capstone-project.md]
   ├─ Learning Objectives
   ├─ Prerequisites
   │  ├─ Chapter 1: Voice-to-Action
   │  ├─ Chapter 2: Cognitive Planning
   │  └─ Module 3: Isaac Sim + Nav2 → /docs/module-3-isaac/
   ├─ Section 1: End-to-End Architecture
   ├─ Section 2: Isaac Sim Integration Testing
   │  └─ Code: Simulation setup
   ├─ Section 3: Multimodal Feedback (Voice + Visual)
   │  ├─ Code: Text-to-speech
   │  └─ Code: Status display
   ├─ Section 4: Benchmark Task Walkthrough
   │  └─ Code: Complete VLA pipeline
   ├─ Section 5: Deployment to Physical Hardware
   │  └─ Code: Hardware-specific configs
   ├─ Summary
   └─ Chapter Navigation
      ├─ ← Previous: Chapter 2
      └─ ↑ Back: Module 4 Index
```

---

## Data Validation Rules

### Frontmatter Validation

All chapters MUST include:
- ✅ `sidebar_position`: Integer 1-3
- ✅ `slug`: String, kebab-case, no file extension
- ✅ `title`: String, includes colon separator
- ✅ `sidebar_label`: String, 3-5 words
- ✅ `description`: String, one sentence, ends with period
- ✅ `tags`: Array with 3-5 strings, lowercase, hyphenated

### Content Validation

Each chapter MUST include:
- ✅ H1 title matching frontmatter `title`
- ✅ "Learning Objectives" H2 section
- ✅ "Prerequisites" H2 section with `:::info` admonition
- ✅ 6-10 main content sections (H2)
- ✅ "Summary and Next Steps" H2 section
- ✅ "Chapter Navigation" bold section
- ✅ Footer with italic chapter identifier

### Code Example Validation

Each code example MUST include:
- ✅ Language specifier (```python, ```bash, etc.)
- ✅ Descriptive comments explaining logic
- ✅ Complete code (includes imports, main function if applicable)
- ✅ Explanation paragraph after code block
- ✅ "How to run" instructions (if applicable)

### Navigation Link Validation

Each chapter MUST include at end:
- ✅ "← Previous" link (except Chapter 1 links to module index)
- ✅ "→ Next" link (except Chapter 3 has no next)
- ✅ "↑ Back to Module 4" link

---

## Summary

This data model defines:
1. **5 Content Entities**: Module, Chapter, Code Example, Content Section, Navigation Link, Admonition
2. **Navigation Structure**: 3 chapters with bidirectional links
3. **Validation Rules**: Ensures consistency across all Module 4 content

**Next**: Create contracts (frontmatter schema, content templates) based on this data model.
