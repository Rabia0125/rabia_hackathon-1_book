# Tasks: RAG Agent Construction & Retrieval Integration

**Input**: Design documents from `/specs/006-rag-agent-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: This feature includes a comprehensive validation test suite as specified in the constitution (Principle VI: Test-Driven Quality).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- Backend: `backend/` directory (extends existing Spec 2 structure)
- Tests: `tests/agent/` directory
- Single-file implementation: `backend/agent.py`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [x] T001 Install OpenAI Agents SDK dependency in backend/pyproject.toml
- [x] T002 Add OPENAI_API_KEY to backend/.env with placeholder
- [x] T003 [P] Create tests/agent/ directory structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core data structures and configuration that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Define AgentConfig dataclass in backend/agent.py (model, temperature, API keys)
- [x] T005 [P] Define Citation dataclass in backend/agent.py (page_title, page_url, module_name)
- [x] T006 [P] Define AgentResponse dataclass in backend/agent.py (answer, citations, timing, confidence)
- [x] T007 Implement load_agent_config() function in backend/agent.py (read env vars)
- [x] T008 Implement validate_agent_config() function in backend/agent.py (fail fast on missing keys)
- [x] T009 Implement validate_query() helper in backend/agent.py (check empty/whitespace)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Question Answering (Priority: P1) 🎯 MVP

**Goal**: Agent accepts questions and generates grounded answers with citations using retrieved context

**Independent Test**: Call agent with "What is ROS 2?" and verify response includes ROS 2 explanation with at least one citation

### Implementation for User Story 1

- [x] T010 [P] [US1] Import retrieve() and related classes from backend/retrieve.py
- [x] T011 [P] [US1] Define RetrievalContext class in backend/agent.py (wrap RetrievalResponse)
- [x] T012 [US1] Implement format_context() in backend/agent.py (convert RetrievalResponse to RetrievalContext)
- [x] T013 [US1] Implement build_prompt() in backend/agent.py (format chunks + grounding instructions)
- [x] T014 [US1] Initialize OpenAI Agent with ModelSettings in backend/agent.py (temperature=0.1, gpt-4o-mini)
- [x] T015 [US1] Implement ask() function in backend/agent.py (sync: validate → retrieve → format → agent.run → response)
- [x] T016 [US1] Add timing instrumentation in ask() function (retrieval_time_ms, generation_time_ms, total_time_ms)
- [x] T017 [US1] Handle empty query error in ask() with ValueError

**Checkpoint**: At this point, User Story 1 should be fully functional - agent answers questions with basic context

---

## Phase 4: User Story 4 - Grounded Response Constraint (Priority: P1)

**Goal**: Agent only answers from retrieved context and declines when information unavailable

**Independent Test**: Ask "What is quantum computing?" and verify agent declines with "couldn't find" message

**Note**: Implementing US4 before US2/US3 because grounding is P1 and foundational to quality

### Implementation for User Story 4

- [x] T018 [US4] Add grounding rules to build_prompt() system instructions in backend/agent.py
- [x] T019 [US4] Implement confidence calculation in ask() (high/low/none based on scores)
- [x] T020 [US4] Handle no results case in ask() - return "couldn't find information" response
- [x] T021 [US4] Handle low relevance case (score < 0.3) - add confidence indicator to response
- [x] T022 [US4] Add adversarial input handling in build_prompt() - reinforce grounding constraints

**Checkpoint**: Agent now strictly enforces grounding and declines out-of-scope questions

---

## Phase 5: User Story 2 - Citation Inclusion (Priority: P2)

**Goal**: Every agent response includes properly formatted citations for all sources used

**Independent Test**: Ask any question, verify response includes `[Source: Page Title]` format citations

### Implementation for User Story 2

- [x] T023 [US2] Implement extract_citations() in backend/agent.py (SearchResult → Citation list)
- [x] T024 [US2] Add citation extraction to ask() function flow
- [x] T025 [US2] Enforce citation format in build_prompt() instructions (`[Source: Page Title]`)
- [x] T026 [US2] Populate AgentResponse.citations field in ask()

**Checkpoint**: All responses now include properly formatted citations

---

## Phase 6: User Story 3 - Multi-Module Query Handling (Priority: P3)

**Goal**: Agent synthesizes information from multiple book modules when needed

**Independent Test**: Ask "How do ROS 2 and Isaac Sim work together?" and verify response cites both ros2 and isaac modules

### Implementation for User Story 3

- [x] T027 [US3] Update build_prompt() to clearly present multi-module context
- [x] T028 [US3] Add multi-module synthesis instruction to agent prompt
- [x] T029 [US3] Verify citation extraction works for chunks from different modules

**Checkpoint**: All user stories complete - agent handles basic Q&A, citations, grounding, and multi-module queries

---

## Phase 7: Error Handling & Robustness

**Purpose**: Comprehensive error handling for all failure modes

- [x] T030 [P] Define RetrievalError exception class in backend/agent.py
- [x] T031 [P] Define AgentError exception class in backend/agent.py
- [x] T032 Wrap retrieve() call with try-except in ask() - catch and raise RetrievalError
- [x] T033 Wrap agent.run() call with try-except in ask() - catch and raise AgentError with retry message
- [x] T034 Add error field to AgentResponse and populate on exceptions

---

## Phase 8: CLI Interface

**Purpose**: Command-line interface for testing and validation

- [x] T035 Implement main() function in backend/agent.py with argparse
- [x] T036 [P] Add --top-k CLI argument
- [x] T037 [P] Add --module CLI argument
- [x] T038 [P] Add --verbose CLI argument
- [x] T039 [P] Add --json output flag
- [x] T040 Add print_results() function for standard text output
- [x] T041 [P] Add print_json() function for JSON output
- [x] T042 Implement CLI error handling with proper exit codes (0, 1, 2, 3, 4)

---

## Phase 9: Async Support (Future-Ready)

**Purpose**: Async API for future FastAPI integration (Spec 4)

- [ ] T043 Implement ask_async() function in backend/agent.py (async version of ask())
- [ ] T044 Use Runner.run() instead of Runner.run_sync() in ask_async()

---

## Phase 10: Validation Test Suite

**Purpose**: Comprehensive validation to verify all success criteria

- [x] T045 Define AGENT_TEST_QUERIES constant in backend/agent.py (6 test queries)
- [x] T046 [P] Define ValidationResult dataclass in backend/agent.py
- [x] T047 [P] Define ValidationReport dataclass in backend/agent.py
- [x] T048 Implement run_agent_validation() in backend/agent.py (execute test suite)
- [x] T049 Implement print_validation_report() in backend/agent.py (format results)
- [x] T050 Add --validate CLI flag to main()
- [x] T051 Connect --validate to run_agent_validation() with pass/fail logic (80% threshold)

---

## Phase 11: Polish & Documentation

**Purpose**: Final refinements and documentation

- [ ] T052 [P] Add docstrings to all public functions in backend/agent.py
- [ ] T053 [P] Add inline comments for complex logic
- [ ] T054 Verify all error messages are user-friendly
- [ ] T055 Run validation suite and confirm 100% pass rate
- [ ] T056 Test CLI with all argument combinations
- [ ] T057 Verify quickstart.md instructions work end-to-end

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (T001-T003) completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (T004-T009) - Core agent functionality
- **User Story 4 (Phase 4)**: Depends on US1 (T010-T017) - Adds grounding constraints
- **User Story 2 (Phase 5)**: Depends on US1 (T010-T017) - Adds citation extraction
- **User Story 3 (Phase 6)**: Depends on US1 + US2 (T010-T026) - Multi-module synthesis
- **Error Handling (Phase 7)**: Depends on US1 complete (T010-T017)
- **CLI (Phase 8)**: Depends on US1 complete (T010-T017)
- **Async (Phase 9)**: Depends on US1 complete (T010-T017)
- **Validation (Phase 10)**: Depends on US1-US4 complete (T010-T022)
- **Polish (Phase 11)**: Depends on all desired features being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (T004-T009) - No dependencies on other stories
- **User Story 4 (P1)**: Depends on US1 for base agent functionality - Adds grounding
- **User Story 2 (P2)**: Depends on US1 for base agent functionality - Adds citations
- **User Story 3 (P3)**: Depends on US1 + US2 - Needs citations to work across modules

### Within Each User Story

- US1: Imports → Data structures → Context formatting → Prompt building → Agent initialization → Main ask() function
- US4: Grounding rules → Confidence calculation → No results handling → Low relevance handling
- US2: Citation extraction → Citation population → Format enforcement
- US3: Multi-module prompt updates → Synthesis instructions

### Parallel Opportunities

- **Phase 1 Setup**: All 3 tasks can run in parallel [P]
- **Phase 2 Foundational**: T005 and T006 can run in parallel [P]
- **Phase 3 US1**: T010 and T011 can run in parallel [P]
- **Phase 7 Error Handling**: T030 and T031 can run in parallel [P]
- **Phase 8 CLI**: T036-T039 and T041 can run in parallel [P]
- **Phase 10 Validation**: T046 and T047 can run in parallel [P]
- **Phase 11 Polish**: T052 and T053 can run in parallel [P]

---

## Parallel Example: User Story 1

```bash
# Launch parallel tasks for User Story 1:
Task T010: "Import retrieve() and related classes from backend/retrieve.py"
Task T011: "Define RetrievalContext class in backend/agent.py"

# Then sequential tasks:
Task T012: "Implement format_context()" (depends on T011)
Task T013: "Implement build_prompt()" (depends on T012)
Task T014: "Initialize OpenAI Agent with ModelSettings"
Task T015: "Implement ask() function" (depends on T013, T014)
Task T016: "Add timing instrumentation" (depends on T015)
Task T017: "Handle empty query error" (depends on T015)
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 4 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009) - CRITICAL
3. Complete Phase 3: User Story 1 (T010-T017)
4. Complete Phase 4: User Story 4 (T018-T022)
5. **STOP and VALIDATE**: Test basic Q&A with grounding
6. Deploy/demo MVP

**MVP Scope**: Agent can answer questions with retrieved context, strictly grounded, declines out-of-scope

### Incremental Delivery

1. Setup + Foundational → Foundation ready (T001-T009)
2. Add User Story 1 → Test independently → Basic Q&A works (T010-T017)
3. Add User Story 4 → Test independently → Grounding enforced (T018-T022) - **MVP!**
4. Add User Story 2 → Test independently → Citations added (T023-T026)
5. Add User Story 3 → Test independently → Multi-module synthesis (T027-T029)
6. Add Error Handling → Robust failures (T030-T034)
7. Add CLI → Command-line interface (T035-T042)
8. Add Validation → Comprehensive test suite (T045-T051)
9. Polish → Production-ready (T052-T057)

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T009)
2. Once Foundational is done:
   - Developer A: User Story 1 (T010-T017)
   - Developer B: CLI Interface (T035-T042) - can start in parallel
   - Developer C: Error Handling (T030-T034) - can start in parallel
3. After US1 complete:
   - Developer A: User Story 4 (T018-T022)
   - Developer B: User Story 2 (T023-T026)
4. After US1+US2 complete:
   - Developer A: User Story 3 (T027-T029)
   - Developer B: Validation Suite (T045-T051)

---

## Task Count Summary

| Phase | Task Count | Can Parallelize |
|-------|------------|-----------------|
| Phase 1: Setup | 3 | 3 (100%) |
| Phase 2: Foundational | 6 | 2 (33%) |
| Phase 3: US1 | 8 | 2 (25%) |
| Phase 4: US4 | 5 | 0 |
| Phase 5: US2 | 4 | 0 |
| Phase 6: US3 | 3 | 0 |
| Phase 7: Error Handling | 5 | 2 (40%) |
| Phase 8: CLI | 8 | 5 (63%) |
| Phase 9: Async | 2 | 0 |
| Phase 10: Validation | 7 | 2 (29%) |
| Phase 11: Polish | 6 | 2 (33%) |
| **TOTAL** | **57** | **18 (32%)** |

**MVP Scope**: T001-T022 (28 tasks) = User Stories 1 + 4 with foundational setup
**Full Feature**: T001-T057 (57 tasks) = All user stories + CLI + validation + polish

---

## Notes

- [P] tasks = different files or independent concerns, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently testable at its checkpoint
- Constitution compliance: Grounding (US4), Citations (US2), Test-Driven Quality (Validation Suite)
- Single-file implementation in `backend/agent.py` keeps complexity low
- Validation suite required by constitution Principle VI
- Commit after each task or logical group of parallel tasks
- Stop at any checkpoint to validate story independently
