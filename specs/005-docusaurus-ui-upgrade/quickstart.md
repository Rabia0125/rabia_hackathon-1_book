# Quickstart: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Time Estimate**: Implementation ready for `/sp.tasks`

## Prerequisites

- Node.js >= 20.0
- Existing Docusaurus project in `frontend_book/`
- Git branch: `005-docusaurus-ui-upgrade`

## Quick Verification

```bash
# Ensure you're on the correct branch
git checkout 005-docusaurus-ui-upgrade

# Navigate to frontend
cd frontend_book

# Install dependencies (if needed)
npm install

# Start development server
npm start
```

## Implementation Order

### Step 1: Theme Variables (custom.css)

Update `frontend_book/src/css/custom.css` with new color palette and typography:

```css
:root {
  --ifm-color-primary: #3578e5;
  --ifm-font-size-base: 17px;
  --ifm-line-height-base: 1.65;
}

[data-theme='dark'] {
  --ifm-color-primary: #6ba3ff;
  --ifm-background-color: #1a1a2e;
}
```

### Step 2: Homepage Hero (index.tsx)

Update hero section with gradient background and proper CTA:

```tsx
<header className={clsx('hero', styles.heroBanner)}>
  <div className="container">
    <h1>Physical AI & Robotics</h1>
    <p>{siteConfig.tagline}</p>
    <Link to="/docs/intro" className="button button--primary button--lg">
      Start Reading
    </Link>
  </div>
</header>
```

### Step 3: Feature Cards (HomepageFeatures)

Replace generic features with book modules:

```tsx
const FeatureList = [
  { title: 'ROS 2 Fundamentals', description: '...', link: '/docs/module-1-ros2' },
  { title: 'Simulation & Digital Twins', description: '...', link: '/docs/module-2-simulation' },
  { title: 'NVIDIA Isaac Platform', description: '...', link: '/docs/module-3-isaac' },
  { title: 'Vision-Language-Action', description: '...', link: '/docs/module-4-vla' },
];
```

### Step 4: Navigation Config (docusaurus.config.ts)

Enable TOC and update footer:

```typescript
docs: {
  tableOfContents: {
    minHeadingLevel: 2,
    maxHeadingLevel: 4,
  },
},
```

## Validation Checklist

After implementation, verify:

- [ ] Light mode colors applied correctly
- [ ] Dark mode colors applied correctly
- [ ] Typography readable (17px base, 1.65 line-height)
- [ ] Homepage hero displays with gradient
- [ ] Feature cards show book modules
- [ ] Sidebar navigation works
- [ ] TOC appears on doc pages
- [ ] Mobile navigation functional
- [ ] No horizontal scroll on mobile
- [ ] Build succeeds (`npm run build`)

## Files Modified

| File | Changes |
|------|---------|
| `src/css/custom.css` | Color palette, typography, layout styles |
| `src/pages/index.tsx` | Hero section content |
| `src/pages/index.module.css` | Homepage styles |
| `src/components/HomepageFeatures/index.tsx` | Feature cards |
| `src/components/HomepageFeatures/styles.module.css` | Card styles |
| `docusaurus.config.ts` | TOC config, footer updates |

## Testing Commands

```bash
# Run development server
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Type check
npm run typecheck
```

## Rollback

If issues occur, revert CSS changes:

```bash
git checkout main -- frontend_book/src/css/custom.css
```

## Next Steps

Run `/sp.tasks` to generate detailed implementation tasks with test cases.
