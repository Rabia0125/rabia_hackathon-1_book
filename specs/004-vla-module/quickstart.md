# Module 4 Documentation Quickstart

**Date**: 2025-12-25
**Purpose**: Guide for adding/editing Module 4: Vision-Language-Action (VLA) content

## Quick Reference

### File Locations

| File | Path | Purpose |
|------|------|---------|
| **Module Index** | `frontend_book/docs/module-4-vla/index.md` | Module overview and introduction |
| **Chapter 1** | `frontend_book/docs/module-4-vla/01-voice-to-action.md` | Voice control with Whisper |
| **Chapter 2** | `frontend_book/docs/module-4-vla/02-cognitive-planning.md` | LLM-based task planning |
| **Chapter 3** | `frontend_book/docs/module-4-vla/03-capstone-project.md` | Integrated VLA system |
| **Sidebar Config** | `frontend_book/sidebars.ts` | Navigation structure |

### Frontmatter Template

```yaml
---
sidebar_position: <1-3>
slug: <chapter-slug>
title: "<Topic>: <Description>"
sidebar_label: "<Short Label>"
description: "<One-sentence description ending with period.>"
tags:
  - <tag-1>
  - <tag-2>
  - <tag-3>
---
```

### Navigation Links Template

```markdown
**Chapter Navigation**:

- ← Previous: [Previous Chapter Title](/docs/module-4-vla/previous-slug)
- → Next: [Next Chapter Title](/docs/module-4-vla/next-slug)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)

---

*Chapter X of Module 4: The AI-Robot Brain (VLA)*
```

---

## Adding a New Chapter

### Step 1: Create the Markdown File

```bash
cd frontend_book/docs/module-4-vla
touch 01-voice-to-action.md  # Or appropriate name
```

### Step 2: Add Frontmatter

See [frontmatter-schema.yaml](./contracts/frontmatter-schema.yaml) for complete specification.

**Example**:
```yaml
---
sidebar_position: 1
slug: voice-to-action
title: "Voice-to-Action: Speech Recognition with Whisper"
sidebar_label: "Voice-to-Action"
description: "Learn to integrate OpenAI Whisper for real-time speech recognition and map voice commands to ROS 2 robot actions."
tags:
  - whisper
  - speech-recognition
  - voice-control
  - ros2-actions
  - intent-recognition
---
```

### Step 3: Add Content Structure

```markdown
# Voice-to-Action: Speech Recognition with Whisper

## Learning Objectives

After completing this chapter, you will be able to:

1. [Objective 1]
2. [Objective 2]
...

## Prerequisites

:::info Before You Begin

- **[Prerequisite]**: [Description] → [Link]

:::

---

## 1. [Main Section]

[Content...]

---

## Summary and Next Steps

Congratulations! You've learned...

### What You Accomplished

- ✅ [Achievement 1]
- ✅ [Achievement 2]

### What's Next?

In **[Next Chapter]**, you'll learn...

[Link to next chapter →]

---

**Chapter Navigation**:

- ← Previous: [Title](./slug)
- → Next: [Title](./slug)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)

---

*Chapter 1 of Module 4: The AI-Robot Brain (VLA)*
```

### Step 4: Update Sidebar Configuration

Edit `frontend_book/sidebars.ts`:

```typescript
{
  type: 'category',
  label: 'Module 4: The AI-Robot Brain (VLA)',
  link: {
    type: 'doc',
    id: 'module-4-vla/index',
  },
  items: [
    'module-4-vla/voice-to-action',      // slug, not filename!
    'module-4-vla/cognitive-planning',
    'module-4-vla/capstone-project',
  ],
},
```

---

## Content Guidelines

### Code Examples

**Python Example**:
````markdown
```python
# Descriptive comment explaining what this does
import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This code creates a ROS 2 node and spins indefinitely.

**To run**:
```bash
ros2 run your_package example_node
```
````

**Bash Example**:
````markdown
```bash
# Install Whisper
pip install openai-whisper

# Test installation
whisper --help
```
````

### Admonitions

Use Docusaurus admonitions for callouts:

```markdown
:::info Hardware Requirements
You'll need an NVIDIA GPU for real-time performance.
:::

:::tip Pro Tip
Use the Whisper `base` model for faster inference on embedded systems.
:::

:::warning Common Gotcha
OpenAI API has rate limits (3 req/min on free tier). Cache common queries.
:::
```

### Cross-Module Links

**Link to other modules** (use absolute paths):
```markdown
[Module 1: ROS 2 Fundamentals](/docs/module-1-ros2/)
[Module 3 Chapter 2: Isaac ROS](/docs/module-3-isaac/isaac-ros)
```

**Link within Module 4**:
```markdown
[Chapter 1: Voice-to-Action](/docs/module-4-vla/voice-to-action)
```

### Tables

```markdown
| Feature | Standard ROS 2 | With VLA |
|---------|----------------|----------|
| **Control Method** | Keyboard/joystick | Natural voice commands |
| **Task Complexity** | Single actions | Multi-step plans |
| **User Experience** | Technical | Conversational |
```

### Images (if needed)

```markdown
![Architecture Diagram](/img/module-4/vla-architecture.png)

*Figure 1: VLA system architecture*
```

Images should be placed in `frontend_book/static/img/module-4/`.

---

## Building and Testing

### Local Development Server

```bash
cd frontend_book
npm install  # First time only
npm start    # Starts dev server at http://localhost:3000
```

Changes are hot-reloaded automatically.

### Production Build

```bash
cd frontend_book
npm run build
```

**Build must complete without errors**. Check for:
- ❌ Broken links
- ❌ Missing frontmatter fields
- ❌ YAML syntax errors
- ❌ Invalid markdown syntax

### Test Build Locally

```bash
cd frontend_book
npm run serve  # Serves the production build at http://localhost:3000
```

### Validation Checklist

Before committing changes:

- [ ] Frontmatter includes all required fields (see schema)
- [ ] `sidebar_position` is unique and correct
- [ ] `slug` is kebab-case and unique
- [ ] `description` ends with a period
- [ ] `tags` array has 3-5 items
- [ ] Navigation links point to correct slugs (not filenames!)
- [ ] All code examples are complete and commented
- [ ] Build completes without errors (`npm run build`)
- [ ] No broken links reported by Docusaurus
- [ ] Content follows constitution principles:
  - ✅ Developer-focused writing
  - ✅ Technical accuracy
  - ✅ Runnable code examples
  - ✅ Clear explanations

---

## Common Issues and Solutions

### Issue 1: Broken Links After Build

**Symptom**: Build fails with "Docusaurus found broken links!"

**Causes**:
- Using relative paths instead of absolute (`./chapter` vs `/docs/module-4-vla/chapter`)
- Linking to filename instead of slug (`01-voice-to-action` vs `voice-to-action`)
- Typo in slug or path

**Solution**:
1. Use absolute paths for all links: `/docs/module-4-vla/slug`
2. Reference slugs from frontmatter, not filenames
3. Run `npm run build` to validate

### Issue 2: Chapter Not Appearing in Sidebar

**Symptom**: Chapter exists but doesn't show in navigation

**Causes**:
- Not added to `sidebars.ts`
- Slug mismatch between frontmatter and sidebar config
- Missing `sidebar_position` or `slug` in frontmatter

**Solution**:
1. Check `frontend_book/sidebars.ts` includes the chapter
2. Verify slug in frontmatter matches slug in `sidebars.ts`
3. Ensure `sidebar_position` is set (1-3 for chapters, 0 for index)

### Issue 3: Code Examples Not Rendering

**Symptom**: Code blocks show as plain text

**Causes**:
- Missing language specifier (````python` vs `````)
- Incorrect indentation
- Unclosed code fence

**Solution**:
1. Always specify language: ````python`, ````bash`, ````yaml`
2. Ensure opening and closing fences are aligned
3. Check for extra backticks or quotes

### Issue 4: Frontmatter YAML Errors

**Symptom**: Build fails with "Error parsing front matter"

**Causes**:
- Incorrect indentation (YAML is whitespace-sensitive)
- Missing quotes around values with special characters
- Mismatched opening/closing `---` markers

**Solution**:
1. Validate YAML syntax (use online YAML validator)
2. Use 2-space indentation for arrays
3. Quote strings with colons, hashes, or special chars
4. Ensure `---` markers are on separate lines

---

## Content Quality Checklist

### Technical Accuracy

- [ ] Code examples are complete and runnable
- [ ] Dependencies are specified with versions (e.g., `openai==1.0.0`)
- [ ] Commands include expected output or success criteria
- [ ] Error handling is demonstrated where appropriate
- [ ] Links to official documentation (ROS 2, Whisper, LangChain)

### Educational Value

- [ ] Learning objectives are clear and specific
- [ ] Prerequisites are explicitly stated with links
- [ ] Concepts are explained before showing code
- [ ] Examples progress from simple to complex
- [ ] Troubleshooting tips are included

### Consistency with Modules 1-3

- [ ] Frontmatter structure matches existing modules
- [ ] Navigation links follow established pattern
- [ ] Code formatting matches (language specifiers, comments)
- [ ] Admonition usage is consistent (info, tip, warning)
- [ ] Terminology matches (e.g., "ROS 2 actions" not "ROS actions")

---

## Getting Help

- **Spec**: See [spec.md](./spec.md) for feature requirements
- **Plan**: See [plan.md](./plan.md) for implementation strategy
- **Research**: See [research.md](./research.md) for existing patterns
- **Data Model**: See [data-model.md](./data-model.md) for content structure
- **Frontmatter Schema**: See [contracts/frontmatter-schema.yaml](./contracts/frontmatter-schema.yaml)
- **Docusaurus Docs**: https://docusaurus.io/docs
- **Markdown Guide**: https://www.markdownguide.org/

---

**Last Updated**: 2025-12-25
**Maintainer**: Claude Code (AI-assisted documentation)
