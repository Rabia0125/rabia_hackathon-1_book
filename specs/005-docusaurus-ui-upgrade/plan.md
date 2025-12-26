# Implementation Plan: Docusaurus UI Upgrade

**Branch**: `005-docusaurus-ui-upgrade` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-docusaurus-ui-upgrade/spec.md`

## Summary

Upgrade the UI of the Physical AI & Robotics Docusaurus book to provide a modern, professional appearance with enhanced typography, improved navigation, responsive design, and polished light/dark themes. Changes are CSS-focused using Infima variables, with React component updates for homepage customization.

## Technical Context

**Language/Version**: TypeScript 5.6, React 19, CSS3
**Primary Dependencies**: Docusaurus 3.9.2, @docusaurus/preset-classic, Infima CSS Framework
**Storage**: N/A (static site, no database)
**Testing**: Visual testing (browser), Lighthouse audits, responsive testing
**Target Platform**: Web (modern browsers), GitHub Pages / Vercel deployment
**Project Type**: Web (frontend only)
**Performance Goals**: Lighthouse accessibility score 90+, page load < 3s
**Constraints**: No new dependencies, CSS variables only (no swizzling), preserve content structure
**Scale/Scope**: 4 modules, ~15 documentation pages, 1 homepage

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-Driven Reproducibility | ✅ PASS | Spec created via `/sp.specify`, plan via `/sp.plan` |
| II. Content Accuracy | ✅ N/A | UI-only changes, no content modifications |
| III. Developer-Focused Writing | ✅ N/A | UI-only changes, no documentation writing |
| IV. Retrieval Transparency | ✅ N/A | No RAG chatbot changes |
| V. Public Reproducibility | ✅ PASS | All changes in public repo, no paid services |
| VI. Test-Driven Quality | ✅ PASS | Visual testing + Lighthouse audits defined |

**Technology Standards Check**:
- ✅ Docusaurus platform maintained
- ✅ No backend changes
- ✅ No secrets required
- ✅ No paid infrastructure

**Gate Result**: PASS - All constitution requirements satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-ui-upgrade/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 output - theming research
├── data-model.md        # Phase 1 output - design tokens
├── quickstart.md        # Phase 1 output - implementation guide
├── contracts/           # Phase 1 output
│   ├── css-variables.yaml    # Design token definitions
│   └── component-specs.yaml  # Component structure specs
├── checklists/
│   └── requirements.md  # Quality validation checklist
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
frontend_book/
├── docusaurus.config.ts     # Site configuration, TOC settings
├── sidebars.ts              # Sidebar configuration (unchanged)
├── src/
│   ├── css/
│   │   └── custom.css       # PRIMARY: All theme variables and styles
│   ├── pages/
│   │   ├── index.tsx        # Homepage component
│   │   └── index.module.css # Homepage styles
│   └── components/
│       └── HomepageFeatures/
│           ├── index.tsx        # Feature cards component
│           └── styles.module.css # Feature card styles
└── static/
    └── img/                 # Static assets (unchanged)
```

**Structure Decision**: Frontend-only changes within existing Docusaurus structure. All UI modifications use the established `src/` directory pattern. No new directories or structural changes required.

## Architecture Decisions

### AD-1: CSS Variables Over Swizzling

**Decision**: Use Infima CSS custom properties for all theming changes.

**Rationale**:
- Maintains upgrade compatibility with Docusaurus updates
- Reduces maintenance burden
- Follows Docusaurus best practices
- Single file (`custom.css`) contains all customizations

**Trade-offs**:
- Less flexibility than swizzled components
- Cannot modify component structure (only styling)

### AD-2: Blue Color Palette

**Decision**: Replace green primary (#2e8555) with blue (#3578e5).

**Rationale**:
- Blue conveys professionalism and technology
- Higher contrast for readability
- Better suited for technical documentation
- Works well in both light and dark modes

### AD-3: System Font Stack

**Decision**: Use system fonts instead of custom web fonts.

**Rationale**:
- Instant loading (no FOUT/FOIT)
- Familiar to users on each platform
- No additional HTTP requests
- Excellent readability across devices

### AD-4: Homepage Feature Cards

**Decision**: Replace default Docusaurus features with book module highlights.

**Rationale**:
- Provides clear entry points to content
- Communicates book structure immediately
- Encourages exploration of all modules

## Implementation Phases

### Phase 1: Theme Foundation
1. Update color palette in `custom.css` (light mode)
2. Update color palette in `custom.css` (dark mode)
3. Configure typography variables
4. Add spacing and layout variables

### Phase 2: Typography & Readability
1. Set base font size and line height
2. Configure heading hierarchy
3. Style code blocks for both themes
4. Add content max-width constraint

### Phase 3: Navigation Enhancement
1. Enable TOC in docusaurus.config.ts
2. Style sidebar active states
3. Enhance navbar appearance
4. Update footer with relevant links

### Phase 4: Homepage Customization
1. Update hero section content and styling
2. Replace feature cards with book modules
3. Add gradient background to hero
4. Style CTA buttons

### Phase 5: Responsive Polish
1. Test and fix mobile navigation
2. Ensure code blocks scroll horizontally
3. Verify touch target sizes
4. Test all breakpoints

### Phase 6: Validation
1. Run Lighthouse accessibility audit
2. Test light/dark mode transitions
3. Verify WCAG contrast ratios
4. Cross-browser testing

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Breaking existing styles | Medium | High | Test each change incrementally, keep backups |
| Accessibility regression | Low | High | Run Lighthouse after each phase |
| Docusaurus update conflict | Low | Medium | Use only CSS variables, avoid swizzling |
| Performance degradation | Low | Low | No new dependencies, CSS-only changes |

## Dependencies

**External**: None (all changes use existing Docusaurus/Infima)

**Internal**:
- Spec: `specs/005-docusaurus-ui-upgrade/spec.md`
- Research: `specs/005-docusaurus-ui-upgrade/research.md`
- Contracts: `specs/005-docusaurus-ui-upgrade/contracts/`

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Lighthouse Accessibility | ≥ 90 | Chrome DevTools audit |
| Lighthouse Performance | ≥ 90 | Chrome DevTools audit |
| Color Contrast (WCAG AA) | ≥ 4.5:1 | Contrast checker tool |
| Mobile Usability | Pass | Manual testing on 320px+ |
| Build Success | Pass | `npm run build` |

## Next Steps

Run `/sp.tasks` to generate detailed implementation tasks with test cases for each phase.
