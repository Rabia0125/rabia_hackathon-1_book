# Data Model: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2025-12-26

## Overview

This feature involves UI/CSS changes only - no database or persistent data storage is required. The "data model" for this feature consists of CSS design tokens (variables) that define the visual system.

---

## Design Token Entities

### Theme Colors

| Token | Light Mode | Dark Mode | Purpose |
|-------|------------|-----------|---------|
| `--ifm-color-primary` | #3578e5 | #6ba3ff | Primary brand color |
| `--ifm-color-primary-dark` | #1d68e1 | #4a8fff | Hover states |
| `--ifm-color-primary-darker` | #1b62d4 | #3985ff | Active states |
| `--ifm-color-primary-darkest` | #1751af | #0066ff | Pressed states |
| `--ifm-color-primary-light` | #4e89e8 | #8cb7ff | Backgrounds |
| `--ifm-color-primary-lighter` | #5a91ea | #9dc1ff | Subtle backgrounds |
| `--ifm-color-primary-lightest` | #80aaef | #cfe0ff | Very subtle backgrounds |
| `--ifm-background-color` | #ffffff | #1a1a2e | Page background |
| `--ifm-background-surface-color` | #f8f9fa | #16213e | Card/surface background |

### Typography Tokens

| Token | Value | Purpose |
|-------|-------|---------|
| `--ifm-font-family-base` | system-ui, -apple-system, ... | Body text font |
| `--ifm-font-family-monospace` | 'JetBrains Mono', 'Fira Code', ... | Code font |
| `--ifm-font-size-base` | 17px | Base body text size |
| `--ifm-line-height-base` | 1.65 | Body text line height |
| `--ifm-heading-font-weight` | 600 | Heading weight |

### Spacing Tokens

| Token | Value | Purpose |
|-------|-------|---------|
| `--ifm-spacing-vertical` | 1rem | Vertical rhythm |
| `--ifm-spacing-horizontal` | 1rem | Horizontal rhythm |
| `--ifm-global-radius` | 0.5rem | Border radius |

### Layout Tokens

| Token | Value | Purpose |
|-------|-------|---------|
| `--doc-sidebar-width` | 280px | Sidebar width |
| `--ifm-container-width` | 1440px | Max container width |
| `--content-max-width` | 75ch | Max content width for readability |

---

## Component Specifications

### Hero Section

```typescript
interface HeroProps {
  title: string;           // "Physical AI & Robotics"
  tagline: string;         // From siteConfig
  ctaText: string;         // "Start Reading"
  ctaLink: string;         // "/docs/intro"
  backgroundGradient: {
    from: string;          // Primary color
    to: string;            // Primary dark
  };
}
```

### Feature Card

```typescript
interface FeatureItem {
  title: string;           // Module name
  description: string;     // Module description
  icon: ReactNode;         // SVG icon component
  link?: string;           // Optional link to module
}
```

**Feature Cards Content**:

| Title | Description |
|-------|-------------|
| ROS 2 Fundamentals | Master the Robot Operating System for real-time robot control |
| Simulation & Digital Twins | Build virtual robots in Gazebo and Unity |
| NVIDIA Isaac Platform | Leverage GPU-accelerated robotics with Isaac Sim and ROS |
| Vision-Language-Action | Implement cutting-edge VLA models for intelligent robots |

### Navigation Structure

```typescript
interface NavigationConfig {
  navbar: {
    title: string;
    logo: { alt: string; src: string };
    items: NavItem[];
  };
  sidebar: {
    collapsible: boolean;
    hideable: boolean;
    autoCollapseCategories: boolean;
  };
  tableOfContents: {
    minHeadingLevel: number;  // 2
    maxHeadingLevel: number;  // 4
  };
}
```

---

## State Transitions

### Theme Mode State

```
[Light Mode] <---> [Dark Mode]
     ^                  ^
     |                  |
     +-- Toggle Button --+
     |                  |
     +-- System Pref ---+
```

**Trigger**: User clicks theme toggle or system preference changes
**Storage**: localStorage key `theme`
**Default**: Respects system preference (`respectPrefersColorScheme: true`)

### Navigation State

```
[Sidebar Collapsed] <---> [Sidebar Expanded]
         ^                       ^
         |                       |
         +--- Toggle Button -----+
         |                       |
         +--- Screen Resize -----+
```

**Mobile**: Sidebar hidden by default, opens as overlay
**Desktop**: Sidebar visible, can be collapsed

---

## Validation Rules

### Color Contrast (WCAG AA)

| Element | Minimum Contrast Ratio |
|---------|----------------------|
| Body text on background | 4.5:1 |
| Large text on background | 3:1 |
| Interactive elements | 3:1 |
| Focus indicators | 3:1 |

### Typography

| Rule | Constraint |
|------|------------|
| Minimum font size | 16px |
| Line height range | 1.4 - 1.8 |
| Max line length | 80 characters |
| Heading scale | 1.25 ratio |

### Touch Targets (Mobile)

| Element | Minimum Size |
|---------|-------------|
| Buttons | 44px × 44px |
| Links in navigation | 44px height |
| Toggle switches | 44px × 24px |

---

## File Structure

```
frontend_book/src/
├── css/
│   └── custom.css           # All CSS custom properties and overrides
├── pages/
│   ├── index.tsx            # Homepage component
│   └── index.module.css     # Homepage styles
└── components/
    └── HomepageFeatures/
        ├── index.tsx        # Feature cards component
        └── styles.module.css # Feature card styles
```
