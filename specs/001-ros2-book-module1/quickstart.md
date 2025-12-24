# Quickstart: Local Development Setup

**Feature**: Module 1 - The Robotic Nervous System (ROS 2)
**Date**: 2025-12-21

## Prerequisites

- Node.js 18.x or higher
- npm or yarn package manager
- Git
- Text editor (VS Code recommended)

## 1. Initialize Docusaurus Project

If starting from scratch, create a new Docusaurus project:

```bash
npx create-docusaurus@latest my-book classic --typescript
cd my-book
```

If the project already exists, skip to step 2.

## 2. Install Dependencies

```bash
npm install
```

## 3. Project Structure Setup

Create the module folder structure:

```bash
mkdir -p docs/module-1-ros2
mkdir -p static/img/module-1
```

## 4. Configure Sidebar

Update `sidebars.js` to include the module:

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        'module-1-ros2/01-ros2-fundamentals',
        'module-1-ros2/02-python-ros-control',
        'module-1-ros2/03-humanoid-urdf',
      ],
    },
  ],
};

module.exports = sidebars;
```

## 5. Create Module Category Configuration

Create `docs/module-1-ros2/_category_.json`:

```json
{
  "label": "Module 1: The Robotic Nervous System",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Learn how ROS 2 acts as the nervous system of humanoid robots."
  }
}
```

## 6. Start Development Server

```bash
npm run start
```

The site will be available at `http://localhost:3000`.

## 7. Build for Production

```bash
npm run build
```

Static files will be generated in `build/` directory.

## 8. Validate Build

```bash
npm run serve
```

Preview the production build locally.

## Common Tasks

### Adding a New Chapter

1. Create a new `.md` file in `docs/module-1-ros2/`
2. Add frontmatter with `sidebar_position`
3. Update `sidebars.js` if using manual sidebar configuration

### Adding Images

1. Place images in `static/img/module-1/`
2. Reference in Markdown: `![Alt text](/img/module-1/image-name.png)`

### Adding Code Blocks

Use fenced code blocks with language hints:

````markdown
```python
print("Hello, ROS 2!")
```
````

### Using Admonitions (Callouts)

```markdown
:::tip
This is a tip
:::

:::warning
This is a warning
:::

:::info Coming from ROS 1?
Key differences are highlighted here.
:::
```

## Troubleshooting

### Port Already in Use

```bash
npm run start -- --port 3001
```

### Build Errors

Check for broken links:

```bash
npm run build -- --locale en
```

### Clear Cache

```bash
npm run clear
npm run start
```

## Next Steps

1. Create `docs/module-1-ros2/index.md` (module overview)
2. Create `docs/module-1-ros2/01-ros2-fundamentals.md` (Chapter 1)
3. Create `docs/module-1-ros2/02-python-ros-control.md` (Chapter 2)
4. Create `docs/module-1-ros2/03-humanoid-urdf.md` (Chapter 3)
5. Add diagrams to `static/img/module-1/`

See `contracts/` folder for detailed chapter outlines.
