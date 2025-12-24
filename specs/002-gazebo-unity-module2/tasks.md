# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-gazebo-unity-module2/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: No automated tests requested. Validation is manual (`npm run build` + content review).

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project:
- **Config files**: Repository root (`docusaurus.config.ts`, `sidebars.ts`, `package.json`)
- **Docs**: `frontend_book/docs/` folder with module subfolders
- **Static assets**: `frontend_book/static/img/module-2/`

---

## Phase 1: Setup (Module Structure Initialization)

**Purpose**: Initialize Module 2 directory structure and configuration

- [x] T001 Create `frontend_book/docs/module-2-simulation/` directory
- [x] T002 Create `frontend_book/docs/module-2-simulation/_category_.json` with module metadata (label: "Module 2: The Digital Twin", position: 2)
- [x] T003 Create `frontend_book/docs/module-2-simulation/index.md` with module overview, learning path, and chapter links
- [x] T004 [P] Create `frontend_book/static/img/module-2/` directory for chapter diagrams
- [x] T005 Update `frontend_book/docs/intro.md` to add Module 2 link in book structure section

---

## Phase 2: Foundational (BLOCKING - Complete Before User Stories)

**Purpose**: Create module landing page and ensure navigation works

**⚠️ CRITICAL**: No chapter content can be written until module structure is in place

- [x] T006 Verify module appears in sidebar navigation with `npm start`
- [x] T007 Verify Module 2 index page renders correctly at `/docs/module-2-simulation`

**Checkpoint**: Module structure ready - chapter content can now be written in parallel

---

## Phase 3: User Story 1 - Master Gazebo Physics Simulation (Priority: P1) 🎯 MVP

**Goal**: Reader can create Gazebo world, spawn humanoid URDF, configure physics, observe realistic behaviors

**Independent Test**: Reader completes Chapter 1 and can launch Gazebo, create world, spawn robot, observe gravity/collisions

### Implementation for User Story 1

- [ ] T008 [US1] Create `frontend_book/docs/module-2-simulation/01-gazebo-simulation.md` with frontmatter (title, sidebar_position: 1, description, tags)
- [ ] T009 [US1] Write Section 1: Introduction - Why Simulation? (~600 words) with Prerequisites callout
- [ ] T010 [US1] Write Section 2: Gazebo Architecture Overview (~800 words)
- [ ] T011 [P] [US1] Create Gazebo architecture diagram (Mermaid) showing Server → Physics Engine → ROS 2 Bridge
- [ ] T012 [US1] Write Section 3: Creating Your First World (~1000 words)
- [ ] T013 [US1] Add Code Example 1: Simple world file (`empty_world.world`) in Section 3
- [ ] T014 [US1] Add Code Example 2: Launch command for Gazebo with custom world in Section 3
- [ ] T015 [US1] Write Section 4: Physics Engines Explained (~900 words) covering ODE, Bullet, Simbody
- [ ] T016 [US1] Add Code Example 3: Physics configuration XML in Section 4
- [ ] T017 [P] [US1] Create physics engine comparison flowchart (Mermaid) in Section 4
- [ ] T018 [US1] Write Section 5: Spawning Robots from URDF (~1200 words)
- [ ] T019 [US1] Add Code Example 4: Bash spawn command in Section 5
- [ ] T020 [US1] Add Code Example 5: Python spawn script in Section 5
- [ ] T021 [US1] Write Section 6: Collision Detection and Contact Forces (~1000 words) with Warning callout about collision mesh performance
- [ ] T022 [US1] Write Section 7: Building Environments (~1200 words)
- [ ] T023 [US1] Add Code Example 6: Adding stairs XML in Section 7
- [ ] T024 [P] [US1] Create world structure hierarchy diagram (Mermaid) in Section 7
- [ ] T025 [US1] Write Section 8: Controlling Robots in Simulation (~1000 words)
- [ ] T026 [US1] Add Code Example 7: Python control node to make humanoid stand in Section 8
- [ ] T027 [US1] Write Section 9: Complete Example - Humanoid Standing (~800 words) with step-by-step walkthrough
- [ ] T028 [US1] Write Section 10: Best Practices and Performance (~600 words) with Tip callout about headless mode
- [ ] T029 [US1] Write Section 11: Summary / Key Takeaways with 5-7 bullet points
- [ ] T030 [US1] Write Section 12: Self-Assessment Questions (5 questions: 2 easy, 2 medium, 1 hard)
- [ ] T031 [US1] Add Additional Resources links (Gazebo docs, SDF spec, gz_ros2_control, model library)
- [ ] T032 [US1] Validate Chapter 1 renders correctly with `npm run build`

**Checkpoint**: Chapter 1 complete - readers can understand and use Gazebo for physics simulation

---

## Phase 4: User Story 2 - Build High-Fidelity Digital Twins in Unity (Priority: P2)

**Goal**: Reader can set up Unity with ROS 2, import humanoid URDF, create photorealistic environment, simulate human-robot interactions

**Independent Test**: Reader completes Chapter 2 and can install Unity, connect to ROS 2, render high-fidelity scenes with human avatars

### Implementation for User Story 2

- [ ] T033 [US2] Create `frontend_book/docs/module-2-simulation/02-unity-digital-twin.md` with frontmatter (title, sidebar_position: 2, description, tags)
- [ ] T034 [US2] Write Section 1: Introduction - Beyond Physics Simulation (~700 words) with Prerequisites callout and Gazebo vs Unity comparison table
- [ ] T035 [US2] Write Section 2: Unity Robotics Hub Overview (~900 words)
- [ ] T036 [P] [US2] Create Unity Robotics Hub architecture diagram (Mermaid) showing Unity → ROS-TCP-Connector → ROS-TCP-Endpoint → ROS 2
- [ ] T037 [US2] Write Section 3: Setting Up Unity for Robotics (~1000 words)
- [ ] T038 [US2] Add Code Example 1: Install ROS-TCP-Endpoint bash commands in Section 3
- [ ] T039 [US2] Add Code Example 2: Unity connection test C# script in Section 3
- [ ] T040 [US2] Write Section 4: Importing URDF Models into Unity (~1200 words) with Warning callout about Y-axis up conversion
- [ ] T041 [US2] Add Code Example 3: URDF import steps (description/screenshots placeholder) in Section 4
- [ ] T042 [US2] Write Section 5: Articulation Bodies - Unity's Physics (~1100 words)
- [ ] T043 [US2] Add Code Example 4: Joint controller C# script in Section 5
- [ ] T044 [P] [US2] Create Articulation Body hierarchy diagram (Mermaid) showing humanoid joint tree in Section 5
- [ ] T045 [US2] Write Section 6: ROS 2 Communication in Unity (~1300 words)
- [ ] T046 [US2] Add Code Example 5: Publisher C# script in Section 6
- [ ] T047 [US2] Add Code Example 6: Subscriber C# script in Section 6
- [ ] T048 [US2] Write Section 7: Creating Photorealistic Environments (~1200 words) with Tip callout about HDRI and GPU instancing
- [ ] T049 [US2] Add Code Example 7: HDRI lighting setup description in Section 7
- [ ] T050 [US2] Write Section 8: Human-Robot Interaction (~1100 words)
- [ ] T051 [US2] Add Code Example 8: Simple follow behavior C# script in Section 8
- [ ] T052 [P] [US2] Create human-robot interaction loop diagram (Mermaid) in Section 8
- [ ] T053 [US2] Write Section 9: Performance Optimization (~900 words) with Warning callout about real-time demands
- [ ] T054 [US2] Write Section 10: Complete Example - Humanoid in Virtual Lab (~1000 words) with expected behavior description
- [ ] T055 [US2] Write Section 11: Gazebo vs Unity - When to Use Each (~600 words) with decision framework
- [ ] T056 [US2] Write Section 12: Summary / Key Takeaways with 7 bullet points and next chapter preview
- [ ] T057 [US2] Write Section 13: Self-Assessment Questions (5 questions: 2 easy, 2 medium, 1 hard)
- [ ] T058 [US2] Add Additional Resources links (Unity Robotics Hub, Articulation Bodies, ROS-TCP-Connector, Unity Learn)
- [ ] T059 [US2] Validate Chapter 2 renders correctly with `npm run build`

**Checkpoint**: Chapter 2 complete - readers can create photorealistic digital twins with Unity

---

## Phase 5: User Story 3 - Simulate Realistic Sensors (Priority: P3)

**Goal**: Reader can configure virtual sensors (LiDAR, depth camera, IMU) in both Gazebo and Unity, visualize outputs, consume via ROS 2

**Independent Test**: Reader completes Chapter 3 and can add sensors to robot models, configure parameters, subscribe to sensor topics

### Implementation for User Story 3

- [ ] T060 [US3] Create `frontend_book/docs/module-2-simulation/03-sensor-simulation.md` with frontmatter (title, sidebar_position: 3, description, tags)
- [ ] T061 [US3] Write Section 1: Introduction - Why Simulate Sensors? (~600 words) with Prerequisites callout
- [ ] T062 [US3] Write Section 2: Sensor Fundamentals (~900 words) covering LiDAR, Depth Camera, IMU basics
- [ ] T063 [P] [US3] Create sensor data flow diagram (Mermaid) showing Physical World → Sensor → ROS 2 Topic → Processing
- [ ] T064 [US3] Write Section 3: LiDAR Simulation in Gazebo (~1200 words)
- [ ] T065 [US3] Add Code Example 1: LiDAR sensor URDF plugin XML in Section 3
- [ ] T066 [US3] Add Code Example 2: LiDAR subscriber Python script in Section 3
- [ ] T067 [US3] Write Section 4: Depth Camera Simulation in Gazebo (~1100 words)
- [ ] T068 [US3] Add Code Example 3: Depth camera URDF plugin XML in Section 4
- [ ] T069 [US3] Add Code Example 4: Depth image processor Python script in Section 4
- [ ] T070 [US3] Write Section 5: IMU Simulation in Gazebo (~1000 words)
- [ ] T071 [US3] Add Code Example 5: IMU sensor URDF plugin XML in Section 5
- [ ] T072 [US3] Add Code Example 6: IMU monitor Python script in Section 5
- [ ] T073 [US3] Write Section 6: Sensors in Unity (~1300 words)
- [ ] T074 [US3] Add Code Example 7: Custom LiDAR raycast C# script in Section 6
- [ ] T075 [US3] Add Code Example 8: Depth camera publisher C# script in Section 6
- [ ] T076 [US3] Write Section 7: Sensor Noise and Realism (~900 words) with Tip callout about gradual noise increase
- [ ] T077 [P] [US3] Create sensor noise model diagram (Mermaid) showing noise pipeline in Section 7
- [ ] T078 [US3] Write Section 8: Sensor Placement on Humanoid (~800 words)
- [ ] T079 [P] [US3] Create humanoid sensor placement diagram (Mermaid or ASCII) in Section 8
- [ ] T080 [US3] Write Section 9: Complete Perception Pipeline (~1200 words)
- [ ] T081 [US3] Add Code Example 9: Sensor fusion node Python script in Section 9
- [ ] T082 [US3] Write Section 10: Gazebo vs Unity for Sensors (~600 words) with comparison table
- [ ] T083 [US3] Write Section 11: Summary / Key Takeaways with 7 bullet points and next steps
- [ ] T084 [US3] Write Section 12: Self-Assessment Questions (5 questions: 2 easy, 2 medium, 1 hard)
- [ ] T085 [US3] Add Additional Resources links (Gazebo sensors, sensor_msgs, Unity Raycasting, sensor fusion tutorials)
- [ ] T086 [US3] Validate Chapter 3 renders correctly with `npm run build`

**Checkpoint**: Chapter 3 complete - readers can simulate complete perception systems

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, consistency checks, and deployment preparation

- [ ] T087 Review all chapters for consistent terminology (Gazebo not "Gazebo Simulator", Unity not "Unity3D", ROS 2 not "ROS2")
- [ ] T088 [P] Verify all code examples have proper language tags and syntax highlighting
- [ ] T089 [P] Verify all Mermaid diagrams render correctly in browser
- [ ] T090 Verify chapter cross-references are accurate (prerequisites, "see Chapter X")
- [ ] T091 Run full Docusaurus build and fix any warnings with `npm run build`
- [ ] T092 [P] Verify all external links work (Gazebo docs, Unity docs, GitHub repos)
- [ ] T093 Verify sidebar navigation order and labels in browser with `npm run serve`
- [ ] T094 Test all chapters load and render correctly in production build

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
3. Add code examples inline as sections are written
4. Create diagrams (Mermaid) inline as sections are written
5. Add self-assessment questions after core content
6. Validate with build command last

### Parallel Opportunities

- T004 (create img directory) can run with T001-T003 (setup)
- T011, T017, T024 (Chapter 1 diagrams) can be created in parallel once sections exist
- T036, T044, T052 (Chapter 2 diagrams) can be created in parallel once sections exist
- T063, T077, T079 (Chapter 3 diagrams) can be created in parallel once sections exist
- T088, T089, T092 (Polish tasks) can run in parallel
- All three chapters (Phases 3-5) can be written in parallel after Phase 2

---

## Parallel Example: All Three Chapters

```bash
# After Phase 2 completes, launch all three chapters in parallel:
# (if multiple authors or using Task agents)

# Author A / Agent A: Chapter 1 (Gazebo)
Tasks: T008-T032

# Author B / Agent B: Chapter 2 (Unity)
Tasks: T033-T059

# Author C / Agent C: Chapter 3 (Sensors)
Tasks: T060-T086

# Then converge for Polish phase
Tasks: T087-T094
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T007)
3. Complete Phase 3: Chapter 1 (T008-T032)
4. **STOP and VALIDATE**: Run `npm run build && npm run serve`
5. Deploy to GitHub Pages if ready

### Incremental Delivery

1. Setup + Foundational → Module structure ready
2. Add Chapter 1 → Test independently → Deploy (MVP!)
3. Add Chapter 2 → Test independently → Deploy
4. Add Chapter 3 → Test independently → Deploy
5. Polish phase → Final validation → Production release

### Parallel Author Strategy

With multiple authors:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Chapter 1 (Gazebo Simulation)
   - Author B: Chapter 2 (Unity Digital Twins)
   - Author C: Chapter 3 (Sensor Simulation)
3. Merge and run Polish phase together

---

## Notes

- [P] tasks = different files, can run in parallel
- [US1/US2/US3] = maps task to specific chapter for traceability
- Each chapter is independently completable and deployable
- Validate build after each chapter completion
- Commit after each section or logical group of tasks
- Stop at any checkpoint to validate and deploy incrementally
- All code examples must be complete and copy-paste ready (even if conceptual/pseudocode for Unity/Gazebo)
- All Mermaid diagrams must be inline in Markdown (no separate image files unless specifically needed)
- Follow Module 1 pattern for consistency

---

## Task Summary

- **Total Tasks**: 94
- **Setup Phase**: 5 tasks
- **Foundational Phase**: 2 tasks
- **User Story 1 (Chapter 1)**: 25 tasks
- **User Story 2 (Chapter 2)**: 27 tasks
- **User Story 3 (Chapter 3)**: 27 tasks
- **Polish Phase**: 8 tasks
- **Parallel Opportunities**: 15 tasks marked with [P]
- **MVP Scope**: T001-T032 (Setup + Foundational + Chapter 1 = 32 tasks)
