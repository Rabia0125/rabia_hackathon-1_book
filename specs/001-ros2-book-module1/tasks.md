# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-book-module1/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: No automated tests requested. Validation is manual (Docusaurus build + content review).

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project:
- **Config files**: Repository root (`docusaurus.config.js`, `sidebars.js`, `package.json`)
- **Docs**: `docs/` folder with module subfolders
- **Static assets**: `static/img/module-1/`

---

## Phase 1: Setup (Docusaurus Initialization)

**Purpose**: Initialize Docusaurus project and configure for book structure

- [x] T001 Initialize Docusaurus project with `npx create-docusaurus@latest my-book classic` in repository root
- [x] T002 Configure `docusaurus.config.js` with book title, tagline, and GitHub Pages deployment settings
- [x] T003 [P] Configure `sidebars.js` with Module 1 navigation structure
- [x] T004 [P] Create `docs/intro.md` with book introduction and overview
- [x] T005 [P] Create `static/img/module-1/` directory for chapter images

---

## Phase 2: Foundational (Module Structure)

**Purpose**: Create module folder structure and shared configuration

**⚠️ CRITICAL**: No chapter content can be written until module structure is in place

- [x] T006 Create `docs/module-1-ros2/` directory
- [x] T007 Create `docs/module-1-ros2/_category_.json` with module metadata and sidebar position
- [x] T008 Create `docs/module-1-ros2/index.md` with module overview, learning path, and chapter links

**Checkpoint**: Module structure ready - chapter content can now be written in parallel

---

## Phase 3: User Story 1 - Learn ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Reader can explain the nervous system analogy and identify when to use topics vs services

**Independent Test**: Reader completes Chapter 1 and can describe nodes, topics, services, and actions using the nervous system analogy

### Implementation for User Story 1

- [x] T009 [US1] Create `docs/module-1-ros2/01-ros2-fundamentals.md` with frontmatter (title, sidebar_position: 1, description, tags)
- [x] T010 [US1] Write Section 1: Introduction - Why ROS 2? (~500 words) including ROS 1 callout
- [x] T011 [US1] Write Section 2: The Nervous System Metaphor (~800 words)
- [ ] T012 [P] [US1] Create nervous system analogy diagram in `static/img/module-1/nervous-system-analogy.png`
- [x] T013 [US1] Write Section 3: Nodes - The Neurons of Your Robot (~1000 words)
- [ ] T014 [P] [US1] Create nodes communication diagram in `static/img/module-1/ros2-nodes.png`
- [x] T015 [US1] Write Section 4: Topics - Sensory Nerves Carrying Data (~1000 words) with CLI code example
- [x] T016 [US1] Write Section 5: Services - Reflexes for Quick Responses (~800 words) with CLI code example
- [x] T017 [US1] Write Section 6: Actions - Motor Commands with Feedback (~800 words) with CLI code example
- [x] T018 [US1] Write Section 7: DDS - The Backbone of Communication (~600 words)
- [ ] T019 [P] [US1] Create ROS 2 architecture diagram in `static/img/module-1/ros2-architecture.png`
- [x] T020 [US1] Write Section 8: Putting It All Together (~400 words) with decision flowchart
- [ ] T021 [P] [US1] Create communication pattern decision flowchart in `static/img/module-1/pattern-flowchart.png`
- [x] T022 [US1] Write Summary/Key Takeaways section
- [x] T023 [US1] Write Self-Assessment Questions section (4 questions)
- [x] T024 [US1] Add Learning Objectives at chapter start (5 objectives)
- [x] T025 [US1] Add Prerequisites callout box at chapter start
- [x] T026 [US1] Validate Chapter 1 renders correctly with `npm run build`

**Checkpoint**: Chapter 1 complete - readers can understand ROS 2 fundamentals

---

## Phase 4: User Story 2 - Connect Python AI to Robot Control (Priority: P2)

**Goal**: Reader can write a publisher/subscriber pair in Python using rclpy

**Independent Test**: Reader completes Chapter 2 and can write a Python node that publishes velocity commands

### Implementation for User Story 2

- [x] T027 [US2] Create `docs/module-1-ros2/02-python-ros-control.md` with frontmatter (title, sidebar_position: 2, description, tags)
- [x] T028 [US2] Write Section 1: Introduction - Bridging AI and Robotics (~400 words)
- [x] T029 [US2] Write Section 2: Setting Up Your Python Environment (~600 words) with package structure code example
- [x] T030 [US2] Write Section 3: Creating Your First Node (~800 words) with MinimalNode code example
- [x] T031 [US2] Write Section 4: Publishing Commands - Controlling Movement (~1200 words) with VelocityPublisher code example
- [ ] T032 [P] [US2] Create publisher data flow diagram in `static/img/module-1/publisher-flow.png`
- [x] T033 [US2] Write Section 5: Subscribing to Sensors - Receiving Data (~1200 words) with ImuSubscriber code example
- [x] T034 [US2] Write Section 6: Calling Services - Request-Response (~1000 words) with ServiceCaller code example
- [x] T035 [US2] Write Section 7: Building an AI Agent (~1500 words) with ReactiveAgent code example
- [ ] T036 [P] [US2] Create agent architecture diagram in `static/img/module-1/agent-architecture.png`
- [x] T037 [US2] Write Section 8: Best Practices and Common Patterns (~500 words)
- [x] T038 [US2] Write Summary/Key Takeaways section
- [x] T039 [US2] Write Self-Assessment Questions section (5 questions)
- [x] T040 [US2] Add Learning Objectives at chapter start (5 objectives)
- [x] T041 [US2] Add Prerequisites callout box referencing Chapter 1
- [x] T042 [US2] Validate Chapter 2 renders correctly with `npm run build`

**Checkpoint**: Chapter 2 complete - readers can write Python ROS 2 nodes

---

## Phase 5: User Story 3 - Model a Humanoid Robot with URDF (Priority: P3)

**Goal**: Reader can interpret an existing humanoid URDF file and identify key joints, links, and sensors

**Independent Test**: Reader completes Chapter 3 and can read a URDF file identifying the robot's kinematic chain

### Implementation for User Story 3

- [x] T043 [US3] Create `docs/module-1-ros2/03-humanoid-urdf.md` with frontmatter (title, sidebar_position: 3, description, tags)
- [x] T044 [US3] Write Section 1: Introduction - Why Model Your Robot? (~400 words)
- [x] T045 [US3] Write Section 2: URDF Basics - XML Structure (~600 words) with minimal URDF skeleton
- [x] T046 [US3] Write Section 3: Links - The Body Parts (~1000 words) with torso link code example
- [ ] T047 [P] [US3] Create link anatomy diagram in `static/img/module-1/link-anatomy.png`
- [x] T048 [US3] Write Section 4: Joints - Connecting the Parts (~1200 words) with shoulder joint code example
- [ ] T049 [P] [US3] Create joint types visualization in `static/img/module-1/joint-types.png`
- [x] T050 [US3] Write Section 5: Humanoid Kinematic Chains (~1000 words)
- [ ] T051 [P] [US3] Create humanoid kinematic tree diagram in `static/img/module-1/kinematic-tree.png`
- [x] T052 [US3] Write Section 6: Sensors in URDF (~800 words) with camera sensor code example
- [x] T053 [US3] Write Section 7: Humanoid-Specific Design Concerns (~800 words) with balance tip callout
- [x] T054 [US3] Write Section 8: Complete Humanoid Example (~700 words) with simplified URDF
- [x] T055 [P] [US3] Create `static/urdf/simple_humanoid.urdf` with full 15-link humanoid model
- [x] T056 [US3] Write Summary/Key Takeaways section
- [x] T057 [US3] Write Self-Assessment Questions section (5 questions)
- [x] T058 [US3] Add Learning Objectives at chapter start (6 objectives)
- [x] T059 [US3] Add Prerequisites callout box referencing Chapters 1-2
- [x] T060 [US3] Validate Chapter 3 renders correctly with `npm run build`

**Checkpoint**: Chapter 3 complete - readers can understand and interpret URDF files

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, consistency checks, and deployment preparation

- [x] T061 Review all chapters for consistent nervous system metaphor usage
- [x] T062 [P] Verify all code examples have syntax highlighting and are copy-paste ready
- [ ] T063 [P] Verify all diagrams have alt text for accessibility (N/A - using ASCII diagrams)
- [x] T064 Verify chapter cross-references are accurate (prerequisites, "see Chapter X")
- [x] T065 Run full Docusaurus build and fix any warnings with `npm run build`
- [x] T066 [P] Update `docs/intro.md` with links to all Module 1 chapters
- [ ] T067 Validate sidebar navigation order and labels in browser
- [ ] T068 Test local preview with `npm run serve` and verify all pages load

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all chapter content
- **User Stories (Phases 3-5)**: All depend on Foundational phase completion
  - Chapters can be written in parallel (if multiple authors)
  - Or sequentially in priority order (Chapter 1 → 2 → 3)
- **Polish (Phase 6)**: Depends on all chapters being complete

### User Story Dependencies

- **User Story 1 (Chapter 1)**: Can start after Phase 2 - No dependencies on other chapters
- **User Story 2 (Chapter 2)**: Can start after Phase 2 - Conceptually follows Chapter 1 but can be written in parallel
- **User Story 3 (Chapter 3)**: Can start after Phase 2 - Conceptually follows Chapters 1-2 but can be written in parallel

### Within Each Chapter

1. Create file with frontmatter first
2. Write sections in order (Introduction → Core content → Summary)
3. Create diagrams in parallel with writing
4. Add learning objectives and prerequisites after core content
5. Validate with build command last

### Parallel Opportunities

- T003, T004, T005 (Setup configuration files)
- T012, T014, T019, T021 (Chapter 1 diagrams)
- T032, T036 (Chapter 2 diagrams)
- T047, T049, T051, T055 (Chapter 3 diagrams and URDF)
- T062, T063, T066 (Polish tasks on different files)
- All three chapters can be written in parallel after Phase 2

---

## Parallel Example: Chapter Diagrams

```bash
# Launch all Chapter 1 diagram tasks in parallel:
Task: "Create nervous system analogy diagram in static/img/module-1/nervous-system-analogy.png"
Task: "Create nodes communication diagram in static/img/module-1/ros2-nodes.png"
Task: "Create ROS 2 architecture diagram in static/img/module-1/ros2-architecture.png"
Task: "Create communication pattern decision flowchart in static/img/module-1/pattern-flowchart.png"
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T008)
3. Complete Phase 3: Chapter 1 (T009-T026)
4. **STOP and VALIDATE**: Run `npm run build && npm run serve`
5. Deploy to GitHub Pages if ready

### Incremental Delivery

1. Setup + Foundational → Site structure ready
2. Add Chapter 1 → Test independently → Deploy (MVP!)
3. Add Chapter 2 → Test independently → Deploy
4. Add Chapter 3 → Test independently → Deploy
5. Polish phase → Final validation → Production release

### Parallel Author Strategy

With multiple authors:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Chapter 1 (ROS 2 Fundamentals)
   - Author B: Chapter 2 (Python + ROS 2)
   - Author C: Chapter 3 (Humanoid URDF)
3. Merge and run Polish phase together

---

## Notes

- [P] tasks = different files, can run in parallel
- [US1/US2/US3] = maps task to specific chapter for traceability
- Each chapter is independently completable and deployable
- Validate build after each chapter completion
- Commit after each section or logical group of tasks
- Stop at any checkpoint to validate and deploy incrementally
- All code examples must be complete and copy-paste ready
- All diagrams must have descriptive filenames and alt text
