# Tasks: Docusaurus UI Upgrade

**Input**: Design documents from `/specs/005-docusaurus-ui-upgrade/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Visual testing via browser and Lighthouse audits (no automated test code required)

**Organization**: Tasks grouped by user story to enable independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5)
- All file paths are relative to `frontend_book/`

## Path Conventions

- **CSS**: `frontend_book/src/css/custom.css`
- **Pages**: `frontend_book/src/pages/`
- **Components**: `frontend_book/src/components/`
- **Config**: `frontend_book/docusaurus.config.ts`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Ensure development environment is ready

- [x] T001 Verify branch is `005-docusaurus-ui-upgrade` and working directory is clean
- [x] T002 Run `npm install` in frontend_book/ to ensure dependencies are current
- [x] T003 Run `npm start` to verify development server starts successfully
- [x] T004 [P] Back up current custom.css to custom.css.backup for rollback

---

## Phase 2: Foundational (Theme Variables)

**Purpose**: Establish CSS custom properties that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Add light mode primary color variables in frontend_book/src/css/custom.css
- [x] T006 Add light mode background and text color variables in frontend_book/src/css/custom.css
- [x] T007 Add dark mode primary color variables in frontend_book/src/css/custom.css
- [x] T008 Add dark mode background and text color variables in frontend_book/src/css/custom.css
- [x] T009 Add typography variables (font-family, font-size, line-height) in frontend_book/src/css/custom.css
- [x] T010 Add spacing and layout variables in frontend_book/src/css/custom.css
- [x] T011 Verify both light and dark modes render without errors in browser

**Checkpoint**: Foundation ready - CSS variables established, user story implementation can begin

---

## Phase 3: User Story 1 - Modern Visual Theme Experience (Priority: P1) MVP

**Goal**: Professional, modern appearance with cohesive color scheme and polished visual elements

**Independent Test**: Open homepage and any doc page in browser; verify modern, professional appearance in both light and dark modes

### Implementation for User Story 1

- [x] T012 [US1] Add border and shadow variables in frontend_book/src/css/custom.css
- [x] T013 [US1] Add transition/animation variables in frontend_book/src/css/custom.css
- [x] T014 [US1] Style navbar with updated colors and shadows in frontend_book/src/css/custom.css
- [x] T015 [US1] Style sidebar background and borders in frontend_book/src/css/custom.css
- [x] T016 [US1] Add smooth theme transition effects in frontend_book/src/css/custom.css
- [x] T017 [US1] Style buttons and interactive elements with hover/focus states in frontend_book/src/css/custom.css
- [x] T018 [US1] Verify visual consistency across homepage and documentation pages

**Checkpoint**: User Story 1 complete - Site has modern, professional appearance

---

## Phase 4: User Story 2 - Enhanced Typography and Readability (Priority: P1)

**Goal**: Comfortable reading experience with clear visual hierarchy for technical content

**Independent Test**: Navigate to any documentation page with mixed content (text, code, images); verify clear typography hierarchy and comfortable reading

### Implementation for User Story 2

- [x] T019 [US2] Set base font-size to 17px and line-height to 1.65 in frontend_book/src/css/custom.css
- [x] T020 [US2] Style heading hierarchy (H1-H6) with appropriate sizes and weights in frontend_book/src/css/custom.css
- [x] T021 [US2] Add content max-width constraint (75ch) for optimal line length in frontend_book/src/css/custom.css
- [x] T022 [US2] Style code blocks with monospace font and proper padding in frontend_book/src/css/custom.css
- [x] T023 [US2] Style inline code with appropriate background and contrast in frontend_book/src/css/custom.css
- [x] T024 [US2] Ensure code syntax highlighting is readable in both themes in frontend_book/src/css/custom.css
- [x] T025 [US2] Add paragraph spacing and margin styles in frontend_book/src/css/custom.css
- [x] T026 [US2] Test readability on a long documentation page (Module 1 or Module 3)

**Checkpoint**: User Story 2 complete - Typography optimized for extended reading

---

## Phase 5: User Story 3 - Improved Navigation Experience (Priority: P2)

**Goal**: Clear book structure navigation with visible current location and section TOC

**Independent Test**: Navigate through all four modules; verify clear hierarchy, current location highlighting, and TOC on long pages

### Implementation for User Story 3

- [x] T027 [US3] Enable table of contents in frontend_book/docusaurus.config.ts (minHeadingLevel: 2, maxHeadingLevel: 4)
- [x] T028 [US3] Style sidebar active link with highlighted background in frontend_book/src/css/custom.css
- [x] T029 [US3] Style sidebar category collapse/expand icons in frontend_book/src/css/custom.css
- [x] T030 [US3] Style table of contents links and active state in frontend_book/src/css/custom.css
- [x] T031 [US3] Add breadcrumb styling if not already styled in frontend_book/src/css/custom.css
- [x] T032 [US3] Update footer links with all modules in frontend_book/docusaurus.config.ts
- [x] T033 [US3] Add GitHub link to navbar in frontend_book/docusaurus.config.ts
- [x] T034 [US3] Test navigation flow: homepage -> Module 1 -> Chapter 2 -> Module 4

**Checkpoint**: User Story 3 complete - Navigation is clear and efficient

---

## Phase 6: User Story 4 - Responsive Mobile Experience (Priority: P2)

**Goal**: Usable reading and navigation on mobile devices

**Independent Test**: Access site on mobile viewport (320px-768px); verify readable content, functional navigation, scrollable code blocks

### Implementation for User Story 4

- [x] T035 [US4] Add mobile viewport meta tag verification in frontend_book/docusaurus.config.ts
- [x] T036 [US4] Style mobile navigation menu toggle in frontend_book/src/css/custom.css
- [x] T037 [US4] Ensure code blocks have horizontal scroll on mobile in frontend_book/src/css/custom.css
- [x] T038 [US4] Set minimum touch target size (44px) for interactive elements in frontend_book/src/css/custom.css
- [x] T039 [US4] Add responsive font-size adjustments for mobile in frontend_book/src/css/custom.css
- [x] T040 [US4] Ensure images are responsive (max-width: 100%) in frontend_book/src/css/custom.css
- [x] T041 [US4] Test on 320px, 375px, 768px viewports using browser DevTools
- [x] T042 [US4] Test landscape orientation on tablet viewport

**Checkpoint**: User Story 4 complete - Mobile experience is functional

---

## Phase 7: User Story 5 - Branded Homepage Experience (Priority: P3)

**Goal**: Homepage clearly communicates book purpose with compelling CTA and module highlights

**Independent Test**: Visit homepage as new user; verify immediate understanding of book subject and clear path to start reading

### Implementation for User Story 5

- [x] T043 [US5] Update hero section title and tagline in frontend_book/src/pages/index.tsx
- [x] T044 [US5] Add gradient background to hero section in frontend_book/src/pages/index.module.css
- [x] T045 [US5] Update CTA button text to "Start Reading" with link to /docs/intro in frontend_book/src/pages/index.tsx
- [x] T046 [US5] Replace default feature items with book modules in frontend_book/src/components/HomepageFeatures/index.tsx
- [x] T047 [US5] Update feature descriptions for ROS 2, Simulation, Isaac, VLA in frontend_book/src/components/HomepageFeatures/index.tsx
- [x] T048 [US5] Style feature cards with hover effects in frontend_book/src/components/HomepageFeatures/styles.module.css
- [x] T049 [US5] Add links from feature cards to respective module pages in frontend_book/src/components/HomepageFeatures/index.tsx
- [x] T050 [US5] Test homepage loads correctly and CTA navigates to intro page

**Checkpoint**: User Story 5 complete - Homepage effectively communicates book value

---

## Phase 8: Polish & Validation

**Purpose**: Final validation and cross-cutting improvements

- [x] T051 Run `npm run build` and verify no build errors
- [x] T052 Run Lighthouse accessibility audit and verify score >= 90
- [x] T053 [P] Verify WCAG AA color contrast ratios using contrast checker tool
- [x] T054 [P] Test keyboard navigation (Tab, Enter, Escape) through all interactive elements
- [x] T055 [P] Test theme toggle transitions (light -> dark -> light)
- [x] T056 Cross-browser test on Chrome, Firefox, Safari (if available)
- [x] T057 [P] Remove custom.css.backup file created in T004
- [x] T058 Final visual review of all pages in both themes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational completion
  - US1 (Theme) and US2 (Typography) can run in parallel
  - US3 (Navigation) can run after US1 or US2
  - US4 (Mobile) can run after US1 and US2
  - US5 (Homepage) can run independently after Foundational
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Depends On | Can Run With |
|-------|------------|--------------|
| US1 (Theme) | Foundational | US2, US5 |
| US2 (Typography) | Foundational | US1, US5 |
| US3 (Navigation) | Foundational | US4, US5 |
| US4 (Mobile) | US1, US2 | US3 |
| US5 (Homepage) | Foundational | US1, US2, US3 |

### Within Each User Story

- CSS changes before config changes
- Style implementation before testing
- Commit after each logical group of changes

### Parallel Opportunities

**Phase 2 (Foundational)**:
- T005 + T006 (light mode colors) can run with T007 + T008 (dark mode colors)

**User Stories**:
- US1 + US2 + US5 can all start simultaneously after Foundational
- US3 + US4 can start once US1 is complete

**Phase 8 (Polish)**:
- T052 + T053 + T054 + T055 can all run in parallel

---

## Parallel Example: User Stories 1 & 2

```bash
# After Foundational phase, launch US1 and US2 together:

# User Story 1 (Theme):
Task: "Add border and shadow variables in frontend_book/src/css/custom.css"
Task: "Style navbar with updated colors and shadows in frontend_book/src/css/custom.css"

# User Story 2 (Typography) - PARALLEL:
Task: "Set base font-size to 17px and line-height to 1.65 in frontend_book/src/css/custom.css"
Task: "Style heading hierarchy (H1-H6) in frontend_book/src/css/custom.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL)
3. Complete Phase 3: User Story 1 (Theme)
4. **STOP and VALIDATE**: Site has modern professional appearance
5. Deploy/demo if ready - This is your MVP!

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 (Theme) → Modern look → Deploy
3. Add US2 (Typography) → Readable content → Deploy
4. Add US3 (Navigation) → Clear wayfinding → Deploy
5. Add US4 (Mobile) → Mobile-friendly → Deploy
6. Add US5 (Homepage) → Branded landing → Deploy
7. Polish → Production-ready

### Suggested Order for Solo Developer

1. Phase 1: Setup (T001-T004)
2. Phase 2: Foundational (T005-T011)
3. Phase 3: US1 Theme (T012-T018)
4. Phase 4: US2 Typography (T019-T026)
5. Phase 5: US3 Navigation (T027-T034)
6. Phase 6: US4 Mobile (T035-T042)
7. Phase 7: US5 Homepage (T043-T050)
8. Phase 8: Polish (T051-T058)

---

## Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Phase 1: Setup | T001-T004 (4 tasks) | Environment preparation |
| Phase 2: Foundational | T005-T011 (7 tasks) | CSS variable foundation |
| Phase 3: US1 Theme | T012-T018 (7 tasks) | Modern visual appearance |
| Phase 4: US2 Typography | T019-T026 (8 tasks) | Readable content |
| Phase 5: US3 Navigation | T027-T034 (8 tasks) | Clear wayfinding |
| Phase 6: US4 Mobile | T035-T042 (8 tasks) | Responsive design |
| Phase 7: US5 Homepage | T043-T050 (8 tasks) | Branded landing page |
| Phase 8: Polish | T051-T058 (8 tasks) | Final validation |
| **Total** | **58 tasks** | |

---

## Notes

- All CSS changes go to `frontend_book/src/css/custom.css` unless otherwise specified
- Use browser DevTools for responsive testing
- Commit after completing each user story phase
- Stop at any checkpoint to validate story independently
- Run `npm run build` periodically to catch errors early
