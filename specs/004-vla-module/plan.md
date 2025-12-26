# Implementation Plan: Module 4 - Vision-Language-Action (VLA) Documentation

**Branch**: `004-vla-module` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-vla-module/spec.md`

## Summary

Create comprehensive documentation for Module 4: Vision-Language-Action (VLA), covering voice-controlled humanoid robots with LLM-based cognitive planning. This module integrates OpenAI Whisper for speech recognition, LLM-based task planning, and combines all previous modules (ROS 2, Simulation, Isaac perception/navigation) into a complete autonomous system.

**Primary Requirement**: Add Module 4 to the Docusaurus documentation structure with 3 chapter pages (Voice-to-Action, Cognitive Planning, Capstone Project), all written in Markdown (.md).

**Technical Approach**: Follow the established Docusaurus + Docusaurus documentation pattern from Modules 1-3, creating an index page and three chapter pages with consistent frontmatter, navigation links, and educational content structure.

## Technical Context

**Language/Version**: Markdown (Docusaurus-flavored), Node.js 24.x (for build/preview)
**Primary Dependencies**: Docusaurus 3.9.2 (static site generator)
**Storage**: Git version control (markdown files in `frontend_book/docs/module-4-vla/`)
**Testing**: Manual content review, Docusaurus build validation (no broken links)
**Target Platform**: Web (static site deployed via GitHub Pages or similar)
**Project Type**: Documentation (Docusaurus site)
**Performance Goals**: Build completes in <30 seconds, pages load in <2 seconds
**Constraints**: Must maintain consistency with Modules 1-3 structure and style
**Scale/Scope**: 4 Markdown files (1 index + 3 chapters), estimated 3,000-4,000 total lines of content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Spec-Driven Reproducibility ✅
- Specification document created (`spec.md`) before implementation
- Implementation plan follows `/sp.plan` workflow
- All outputs are reproducible from specs

### II. Content Accuracy ✅
- Module 4 content will cover established technologies (Whisper, LLMs, ROS 2)
- Technical concepts will reference official documentation
- Code examples will be validated against ROS 2 and Isaac ROS standards

### III. Developer-Focused Writing ✅
- Content targets "AI and robotics developers integrating LLMs with humanoid robots" (from spec)
- Will include runnable code examples with comments
- Technical terms will be defined clearly with practical examples

### IV. Retrieval Transparency N/A
- This phase creates content; RAG retrieval will be implemented in future phase

### V. Public Reproducibility ✅
- Markdown files are publicly accessible
- Docusaurus build is reproducible from source
- Free-tier deployment (GitHub Pages)

### VI. Test-Driven Quality ✅
- Docusaurus build validation ensures no broken links
- Content structure validated against acceptance criteria
- Manual review for technical accuracy

**GATE RESULT**: ✅ PASS - All applicable constitution principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/004-vla-module/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (already created)
├── research.md          # Phase 0 output (Docusaurus patterns, content structure)
├── data-model.md        # Phase 1 output (content entities, navigation structure)
├── quickstart.md        # Phase 1 output (how to add/edit Module 4 content)
├── contracts/           # Phase 1 output (frontmatter schemas, content templates)
│   └── frontmatter-schema.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend_book/
├── docs/
│   └── module-4-vla/        # NEW: Module 4 documentation
│       ├── index.md          # Module overview and introduction
│       ├── 01-voice-to-action.md  # Chapter 1: Whisper integration
│       ├── 02-cognitive-planning.md  # Chapter 2: LLM task planning
│       └── 03-capstone-project.md    # Chapter 3: Integrated VLA system
├── sidebars.ts              # UPDATED: Add Module 4 navigation
└── docusaurus.config.ts     # No changes needed (uses auto-sidebar)
```

**Structure Decision**: This follows the **Documentation Project** pattern established by Modules 1-3. Each module has an index page and 3 chapters with slug-based routing and consistent frontmatter structure.

## Complexity Tracking

No constitution violations to justify - this is a straightforward documentation feature following established patterns.

## Phase 0: Research

### Research Questions

1. **Q1: Docusaurus frontmatter structure for Module 4**
   - **What to research**: Analyze existing frontmatter in Modules 1-3 to ensure consistency
   - **Why**: Frontmatter controls sidebar positioning, slugs, titles, descriptions, and tags
   - **Output**: Documented frontmatter pattern with examples

2. **Q2: Content structure and depth for VLA topics**
   - **What to research**: Best practices for explaining Whisper, LLM integration, and ROS 2 action mapping
   - **Why**: Ensures content is educational, practical, and matches target audience skill level
   - **Output**: Content outline with section headings for each chapter

3. **Q3: Navigation links and cross-references**
   - **What to research**: How Modules 1-3 link between chapters and reference each other
   - **Why**: Maintains consistent navigation UX across all modules
   - **Output**: Link structure pattern (relative vs absolute paths, chapter navigation format)

4. **Q4: Code example format and testing**
   - **What to research**: How to present ROS 2 + Python + LLM code examples effectively
   - **Why**: Code examples must be runnable, well-commented, and follow ROS 2 best practices
   - **Output**: Code example templates for voice integration, LLM calls, and action execution

### Research Tasks

#### Task 1: Analyze Existing Module Structure
**Objective**: Document the exact frontmatter pattern, content structure, and navigation used in Modules 1-3

**Actions**:
1. Read frontmatter from `module-3-isaac/index.md` and all 3 chapters
2. Document required fields: `sidebar_position`, `slug`, `title`, `sidebar_label`, `description`, `tags`
3. Analyze navigation links (← Previous, → Next, ↑ Back to Module)
4. Check how module index links to chapters and how chapters link to prerequisites

**Output**: `research.md` section documenting:
- Frontmatter schema with examples
- Navigation link patterns
- Content structure (Learning Objectives, Prerequisites, sections, Summary)

#### Task 2: Content Depth Research
**Objective**: Define appropriate depth for VLA topics (Whisper, LLMs, ROS 2 integration)

**Actions**:
1. Review OpenAI Whisper documentation for key concepts to explain
2. Review LangChain/LlamaIndex patterns for LLM orchestration in robotics
3. Identify ROS 2 action interfaces relevant to humanoid control
4. Determine balance between theory and hands-on examples

**Output**: `research.md` section with:
- Content outline for each of the 3 chapters
- Key concepts to cover per chapter
- Example types (code snippets, diagrams, use cases)

#### Task 3: Cross-Module Integration Research
**Objective**: Ensure Module 4 properly references and integrates with Modules 1-3

**Actions**:
1. Identify prerequisite knowledge from Module 1 (ROS 2, Python, actions)
2. Identify prerequisite tools from Module 3 (Isaac Sim, Isaac ROS, Nav2)
3. Document how VLA builds on previous modules
4. Plan integration examples showing voice → LLM → perception → navigation flow

**Output**: `research.md` section with:
- Prerequisite mapping (which Module 1-3 concepts are required)
- Integration touchpoints (how Module 4 uses Module 3 outputs)
- Example scenarios combining all modules

**Research Deliverable**: `research.md` containing all findings organized by research question

## Phase 1: Design

### Data Model

Since this is documentation (not software), the "data model" describes the **content entities** and **navigation structure**.

#### Content Entities

**Entity: Module**
- **Attributes**: Title, description, duration, chapter count, prerequisites
- **Purpose**: Top-level organizational unit (Module 4: VLA)
- **Relationships**: Contains 3 Chapters, depends on Modules 1-3

**Entity: Chapter**
- **Attributes**: Title, sidebar_position, slug, description, tags, learning objectives
- **Purpose**: Individual learning unit within a module
- **Relationships**: Belongs to Module 4, links to previous/next chapters

**Entity: Code Example**
- **Attributes**: Language (Python, bash), code block, explanation, expected output
- **Purpose**: Runnable examples demonstrating concepts
- **Relationships**: Embedded within Chapter content

**Entity: Navigation Link**
- **Attributes**: Label (← Previous, → Next, ↑ Back), target URL, link text
- **Purpose**: User navigation between chapters and modules
- **Relationships**: Links between Chapters and Module index

#### Navigation Structure

```text
Module 4: The AI-Robot Brain (VLA)
├── Introduction (index.md)
│   ├── Why VLA?
│   ├── Prerequisites (Modules 1-3)
│   └── Learning Path
│
├── Chapter 1: Voice-to-Action (01-voice-to-action.md)
│   ├── Whisper Installation
│   ├── Speech-to-Text Pipeline
│   ├── Intent Recognition
│   ├── ROS 2 Action Mapping
│   └── Testing and Debugging
│
├── Chapter 2: Cognitive Planning (02-cognitive-planning.md)
│   ├── LLM Integration (GPT-4, Claude, Llama)
│   ├── Task Decomposition
│   ├── Plan Validation
│   ├── Dynamic Replanning
│   └── Safety Constraints
│
└── Chapter 3: Capstone Project (03-capstone-project.md)
    ├── End-to-End Integration
    ├── Isaac Sim Testing
    ├── Multimodal Feedback
    ├── Benchmark Task Execution
    └── Deployment to Physical Hardware
```

### Contracts

#### Frontmatter Schema

```yaml
# contracts/frontmatter-schema.yaml
# Schema for all Module 4 chapter frontmatter

ModuleIndex:
  sidebar_position: 0
  title: "Module 4: The AI-Robot Brain (VLA)"
  description: "Learn to integrate voice commands, LLM-based planning, and autonomous robot control for humanoid robots"

Chapter:
  sidebar_position: <integer 1-3>
  slug: <kebab-case-name>
  title: "<Full Chapter Title>"
  sidebar_label: "<Short Label>"
  description: "<One-sentence description for SEO>"
  tags:
    - <relevant-tag-1>
    - <relevant-tag-2>
    - <relevant-tag-3>

Example:
  ---
  sidebar_position: 1
  slug: voice-to-action
  title: "Voice-to-Action: Speech Recognition with Whisper"
  sidebar_label: "Voice-to-Action"
  description: "Learn to integrate OpenAI Whisper for real-time speech recognition and map voice commands to ROS 2 robot actions"
  tags:
    - whisper
    - speech-recognition
    - voice-control
    - ros2-actions
  ---
```

#### Content Template Structure

```markdown
# contracts/chapter-template.md

# [Chapter Title]

## Learning Objectives

After completing this chapter, you will be able to:

1. [Objective 1]
2. [Objective 2]
3. [Objective 3]
...

## Prerequisites

:::info Before You Begin

- **[Prerequisite 1]**: [Description] → [Link to Module X]
- **[Prerequisite 2]**: [Description]
- **[Hardware/Software]**: [Requirements]

:::

---

## 1. [Main Section Title]

[Introductory paragraph explaining the section]

### [Subsection Title]

[Content with explanations]

```language
# Code example
```

[Explanation of code]

:::tip [Tip Title]
[Helpful tip or best practice]
:::

:::warning [Warning Title]
[Important caution or gotcha]
:::

---

## Summary and Next Steps

Congratulations! You've learned how to [summary of what was covered].

### What You Accomplished

- ✅ [Achievement 1]
- ✅ [Achievement 2]
- ✅ [Achievement 3]

### Key Takeaways

1. **[Key Point 1]**: [Explanation]
2. **[Key Point 2]**: [Explanation]
3. **[Key Point 3]**: [Explanation]

### What's Next?

In **[Next Chapter]**, you'll learn how to:

- [Preview of next chapter topic 1]
- [Preview of next chapter topic 2]

[Link to next chapter →]

---

**Chapter Navigation**:

- ← Previous: [Previous Chapter Title](./previous-slug)
- → Next: [Next Chapter Title](./next-slug)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)

---

*Chapter X of Module 4: The AI-Robot Brain (VLA)*
```

### Quickstart Guide

```markdown
# specs/004-vla-module/quickstart.md

# Module 4 Documentation Quickstart

## Adding/Editing Module 4 Content

### File Locations

- **Module Index**: `frontend_book/docs/module-4-vla/index.md`
- **Chapter 1**: `frontend_book/docs/module-4-vla/01-voice-to-action.md`
- **Chapter 2**: `frontend_book/docs/module-4-vla/02-cognitive-planning.md`
- **Chapter 3**: `frontend_book/docs/module-4-vla/03-capstone-project.md`
- **Sidebar Config**: `frontend_book/sidebars.ts`

### Frontmatter Requirements

All chapters must include:

```yaml
---
sidebar_position: <1-3>
slug: <chapter-slug>
title: "Full Chapter Title"
sidebar_label: "Short Label"
description: "SEO description"
tags:
  - tag1
  - tag2
---
```

### Navigation Links

At the end of each chapter:

```markdown
**Chapter Navigation**:

- ← Previous: [Title](./slug)
- → Next: [Title](./slug)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)
```

### Building and Testing

```bash
# Install dependencies
cd frontend_book
npm install

# Start dev server
npm start

# Build for production
npm run build

# Test build
npm run serve
```

### Content Guidelines

1. **Code Examples**: Use Python for ROS 2/LLM code, bash for shell commands
2. **Admonitions**: Use `:::info`, `:::tip`, `:::warning` for callouts
3. **Links**: Use absolute paths for cross-module links (`/docs/module-X/`)
4. **Images**: Place in `frontend_book/static/img/module-4/` (if needed)

### Validation Checklist

- [ ] Frontmatter includes all required fields
- [ ] Navigation links are correct (previous/next/up)
- [ ] Code examples are complete and commented
- [ ] Build completes without errors (`npm run build`)
- [ ] No broken links reported by Docusaurus
- [ ] Content follows constitution principles (accuracy, developer-focused)
```

## Phase 2: Implementation Notes

**Note**: Phase 2 (task generation) is handled by `/sp.tasks` command. This plan provides the foundation for task creation.

### Implementation Sequence

The tasks will be created in this order:

1. **Setup Tasks**:
   - Create `module-4-vla/` directory
   - Copy frontmatter schema and content template to contracts/
   - Update `sidebars.ts` with Module 4 entry

2. **Content Creation Tasks** (one per file):
   - Create `index.md` (Module 4 overview)
   - Create `01-voice-to-action.md` (Chapter 1)
   - Create `02-cognitive-planning.md` (Chapter 2)
   - Create `03-capstone-project.md` (Chapter 3)

3. **Validation Tasks**:
   - Run `npm run build` to check for broken links
   - Manual content review for accuracy
   - Test navigation links between chapters

### Acceptance Criteria (for `/sp.tasks`)

Each chapter must include:
- ✅ Frontmatter with all required fields
- ✅ Learning Objectives section
- ✅ Prerequisites section with links to Modules 1-3
- ✅ At least 3 main sections with subsections
- ✅ Minimum 2 code examples per chapter
- ✅ Summary and Next Steps section
- ✅ Chapter Navigation links
- ✅ Estimated 800-1200 lines of content per chapter

Module index must include:
- ✅ Module overview and motivation
- ✅ Learning path diagram/flowchart
- ✅ Prerequisites section linking to Modules 1-3
- ✅ Hardware requirements table
- ✅ Links to all 3 chapters
- ✅ Estimated 250-350 lines of content

## Risk Mitigation

### Risk 1: Content Depth Mismatch
**Risk**: Module 4 content may be too shallow or too deep for target audience (AI/robotics developers)

**Mitigation**:
- Research phase includes analyzing OpenAI Whisper and LangChain documentation for appropriate depth
- Each chapter includes "Prerequisites" section to set expectations
- Code examples are complete and runnable to provide hands-on learning

### Risk 2: Broken Cross-Module References
**Risk**: Links to Modules 1-3 may break or point to incorrect content

**Mitigation**:
- Use absolute paths for cross-module links (`/docs/module-1-ros2/`)
- Docusaurus build validation catches broken links
- Manual testing of all navigation links

### Risk 3: LLM API Changes
**Risk**: OpenAI/Anthropic API changes may invalidate code examples

**Mitigation**:
- Include API version numbers in code examples (e.g., `openai==1.0.0`)
- Document fallback to local LLMs (Llama, Mistral) as alternative
- Regular content updates planned post-launch

### Risk 4: Inconsistent Style with Modules 1-3
**Risk**: Module 4 may not match the tone, style, or structure of existing modules

**Mitigation**:
- Research phase explicitly analyzes Modules 1-3 for patterns
- Frontmatter schema and content template enforce consistency
- Manual review against Module 3 (most recent) as benchmark

## Summary

This plan establishes the foundation for creating Module 4: Vision-Language-Action documentation. The research phase will analyze existing module patterns and document the exact structure needed. The design phase defines content entities, navigation structure, and contracts (frontmatter schema, content templates). Implementation tasks will be generated by `/sp.tasks` based on this plan.

**Next Command**: `/sp.tasks` to generate ordered, testable tasks for Module 4 content creation.
