# Research: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2025-12-26
**Status**: Complete

## Executive Summary

This research documents best practices and technical decisions for upgrading the UI of a Docusaurus 3.9.2 project. The focus is on CSS customization, theming, typography, and responsive design within the Infima CSS framework.

---

## Research Area 1: Docusaurus Theming Best Practices

### Decision
Use CSS custom properties (CSS variables) through Infima framework for all theming changes.

### Rationale
- Docusaurus uses Infima CSS framework with built-in CSS custom properties
- Modifying `src/css/custom.css` is the recommended approach
- Avoids need for swizzling components (which complicates upgrades)
- Supports both light and dark themes via `[data-theme='dark']` selector

### Alternatives Considered
| Alternative | Why Rejected |
|-------------|--------------|
| Swizzle all theme components | Increases maintenance burden, breaks on Docusaurus updates |
| External CSS framework (Tailwind) | Conflicts with Infima, adds complexity |
| Inline styles | Not maintainable, doesn't support theming |

### Key Findings
- Primary color variables: `--ifm-color-primary` and related shades
- Typography variables: `--ifm-font-family-base`, `--ifm-font-size-base`, `--ifm-line-height-base`
- Spacing variables: `--ifm-spacing-vertical`, `--ifm-spacing-horizontal`
- All customizations should go in `src/css/custom.css`

---

## Research Area 2: Color Palette for Technical Documentation

### Decision
Use a professional blue-based color palette with high contrast for readability.

### Rationale
- Blue conveys trust, professionalism, and technology
- Better contrast than green for long-form reading
- Aligns with robotics/AI domain aesthetics
- Works well in both light and dark modes

### Color Palette
```css
/* Light Mode */
--ifm-color-primary: #3578e5;      /* Primary blue */
--ifm-color-primary-dark: #1d68e1;
--ifm-color-primary-darker: #1b62d4;
--ifm-color-primary-darkest: #1751af;
--ifm-color-primary-light: #4e89e8;
--ifm-color-primary-lighter: #5a91ea;
--ifm-color-primary-lightest: #80aaef;

/* Dark Mode */
--ifm-color-primary: #6ba3ff;
--ifm-color-primary-dark: #4a8fff;
--ifm-color-primary-darker: #3985ff;
--ifm-color-primary-darkest: #0066ff;
--ifm-color-primary-light: #8cb7ff;
--ifm-color-primary-lighter: #9dc1ff;
--ifm-color-primary-lightest: #cfe0ff;
```

### Alternatives Considered
| Alternative | Why Rejected |
|-------------|--------------|
| Keep green (#2e8555) | Less professional for technical docs, lower contrast |
| Purple/violet | Too creative/playful for robotics domain |
| Red/orange | Associated with errors/warnings |

---

## Research Area 3: Typography System

### Decision
Use system font stack with optimized line height and max-width for readability.

### Rationale
- System fonts load instantly (no FOUT/FOIT)
- Inter/system-ui are highly readable for technical content
- Line height of 1.65 optimal for body text
- Max-width of 75ch prevents eye strain on wide screens

### Typography Settings
```css
--ifm-font-family-base: system-ui, -apple-system, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
--ifm-font-family-monospace: 'JetBrains Mono', 'Fira Code', Consolas, Monaco, 'Courier New', monospace;
--ifm-font-size-base: 17px;
--ifm-line-height-base: 1.65;
--ifm-heading-font-weight: 600;
```

### Key Findings
- Headings should use font-weight 600-700 for clear hierarchy
- Code blocks need slightly larger font size (15-16px) for readability
- Line height for code should be tighter (1.4-1.5)
- Letter-spacing can be slightly increased for headings

---

## Research Area 4: Navigation Enhancement

### Decision
Enable table of contents (TOC) on doc pages and enhance sidebar styling.

### Rationale
- TOC helps users navigate long technical chapters
- Sticky TOC keeps context visible while scrolling
- Sidebar icons/colors help distinguish modules
- Breadcrumbs aid in understanding location

### Implementation Approach
1. Enable TOC in docusaurus.config.ts:
   ```javascript
   docs: {
     sidebarPath: './sidebars.ts',
     tableOfContents: {
       minHeadingLevel: 2,
       maxHeadingLevel: 4,
     },
   }
   ```
2. Style sidebar with category icons via CSS
3. Enhance active link styling for clear location indication

### Alternatives Considered
| Alternative | Why Rejected |
|-------------|--------------|
| Custom TOC component | Unnecessary, built-in TOC is sufficient |
| Breadcrumb swizzle | Default breadcrumbs work well with styling only |

---

## Research Area 5: Responsive Design

### Decision
Use Infima's built-in breakpoints with custom mobile navigation enhancements.

### Rationale
- Infima has responsive breakpoints at 576px, 768px, 996px, 1440px
- Mobile navigation already works, needs styling refinement
- Code blocks need horizontal scroll on mobile
- Touch targets need minimum 44px size

### Breakpoint Strategy
```css
/* Mobile-first approach */
@media (min-width: 576px) { /* Small tablets */ }
@media (min-width: 768px) { /* Tablets */ }
@media (min-width: 996px) { /* Desktop */ }
@media (min-width: 1440px) { /* Large desktop */ }
```

### Key Findings
- Default mobile menu works well, needs color/spacing adjustments
- Code blocks should use `overflow-x: auto` on mobile
- Images should use `max-width: 100%` and `height: auto`
- Font size can be slightly smaller on mobile (16px vs 17px)

---

## Research Area 6: Homepage Enhancement

### Decision
Customize hero section and feature cards with project-specific content.

### Rationale
- Default Docusaurus features are generic
- Hero should clearly communicate book purpose
- Feature cards should highlight key modules
- Call-to-action should lead directly to first chapter

### Implementation Approach
1. Update `src/pages/index.tsx`:
   - Custom hero with gradient background
   - Relevant tagline and CTA button
2. Update `src/components/HomepageFeatures/index.tsx`:
   - Replace default features with book modules
   - Add icons relevant to robotics/AI
3. Style in `src/css/custom.css`:
   - Hero gradient
   - Feature card hover effects
   - CTA button styling

### Alternatives Considered
| Alternative | Why Rejected |
|-------------|--------------|
| Remove homepage, redirect to docs | Loses marketing/landing page value |
| Third-party landing page template | Adds complexity, may conflict with Docusaurus |

---

## Research Area 7: Dark Mode Enhancement

### Decision
Create polished dark mode with proper contrast and reduced eye strain.

### Rationale
- Many developers prefer dark mode
- Technical reading sessions can be long
- Dark mode reduces eye strain in low-light environments
- Must maintain WCAG AA contrast ratios

### Dark Mode Enhancements
```css
[data-theme='dark'] {
  --ifm-background-color: #1a1a2e;
  --ifm-background-surface-color: #16213e;
  --ifm-color-content: #e8e8e8;
  --ifm-code-background: #0f0f23;
  --ifm-toc-border-color: #2a2a4a;
}
```

### Key Findings
- Avoid pure black (#000) backgrounds - causes eye strain
- Use slightly blue-tinted dark backgrounds for tech feel
- Code blocks need different background than page
- Links need higher saturation in dark mode for visibility

---

## Files to Modify

| File | Changes |
|------|---------|
| `frontend_book/src/css/custom.css` | All theme variables, typography, layout styles |
| `frontend_book/src/pages/index.tsx` | Hero section content, layout structure |
| `frontend_book/src/pages/index.module.css` | Homepage-specific styles |
| `frontend_book/src/components/HomepageFeatures/index.tsx` | Feature cards content |
| `frontend_book/src/components/HomepageFeatures/styles.module.css` | Feature card styles |
| `frontend_book/docusaurus.config.ts` | TOC settings, navbar enhancements |

## Dependencies

No new dependencies required. All changes use:
- Existing Docusaurus 3.9.2 features
- Infima CSS framework (included with Docusaurus)
- React 19 (already installed)

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Breaking existing layout | Test each change incrementally |
| Docusaurus update conflicts | Use CSS variables instead of swizzling |
| Accessibility regression | Validate contrast ratios, test keyboard navigation |
| Performance impact | No new dependencies, CSS-only changes |
