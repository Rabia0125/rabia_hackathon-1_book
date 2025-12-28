# Tasks: FastAPI Backend & Frontend Integration

**Input**: Design documents from `/specs/007-fastapi-backend-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/openapi.yaml

**Tests**: Tests are NOT explicitly requested in the feature specification. Test tasks are included for validation but can be implemented after core functionality.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a web application with:
- Backend: `backend/` (Python/FastAPI)
- Frontend: `frontend_book/` (Docusaurus/React)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Install FastAPI dependencies in backend/pyproject.toml (fastapi, uvicorn, python-multipart)
- [X] T002 [P] Create backend/.env.example with required environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, CORS_ORIGINS, API_HOST, API_PORT, LOG_LEVEL)
- [X] T003 [P] Create backend/tests/ directory structure with __init__.py
- [X] T004 [P] Create backend/tests/conftest.py with pytest fixtures for FastAPI TestClient

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Create Pydantic data models in backend/api.py: ChatRequest, ChatResponse, Citation, ErrorDetail, ErrorResponse, HealthStatus
- [X] T006 [P] Initialize FastAPI application in backend/api.py with CORS middleware configuration
- [X] T007 [P] Implement custom exception classes in backend/api.py: ValidationError, AgentError, RetrievalError
- [X] T008 [P] Register FastAPI exception handlers for custom exceptions in backend/api.py
- [X] T009 [P] Implement logging configuration in backend/api.py (structlog or standard logging with JSON formatters)
- [X] T010 Implement /health endpoint (liveness probe) in backend/api.py
- [X] T011 Implement /ready endpoint (readiness probe) with agent, retrieval, and env checks in backend/api.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book-Wide Chat Query (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask natural language questions and receive grounded answers with citations from the book content

**Independent Test**: Submit a query "What is ROS 2?" via POST /chat and verify response contains answer with citations from the book

### Implementation for User Story 1

- [X] T012 [US1] Implement POST /chat endpoint handler in backend/api.py that accepts ChatRequest
- [X] T013 [US1] Add request validation logic in /chat handler to enforce FR-005 (empty query, max 2000 chars)
- [X] T014 [US1] Integrate agent.ask() call in /chat handler with query parameter
- [X] T015 [US1] Map agent.AgentResponse to ChatResponse model in /chat handler
- [X] T016 [US1] Add error handling in /chat handler for AgentError (HTTP 500) and RetrievalError (HTTP 503)
- [X] T017 [US1] Add timing metrics calculation (retrieval_time_ms, generation_time_ms, total_time_ms) in /chat handler
- [X] T018 [US1] Add request/response logging in /chat handler for FR-009 compliance

### Tests for User Story 1

- [X] T019 [P] [US1] Create backend/tests/test_api.py with unit tests for ChatRequest validation (empty query, too long, valid)
- [X] T020 [P] [US1] Add integration test for /chat endpoint with valid query in backend/tests/test_integration.py
- [X] T021 [P] [US1] Add integration test for /chat endpoint error cases (empty query, agent failure) in backend/tests/test_integration.py

**Checkpoint**: At this point, User Story 1 should be fully functional - readers can ask questions and get answers with citations

---

## Phase 4: User Story 2 - Selected Text Context Query (Priority: P2)

**Goal**: Enable readers to highlight text and ask follow-up questions specifically about that selected content

**Independent Test**: Select text from a page, submit query with selected_text parameter, verify response is contextually relevant to the selection

### Implementation for User Story 2

- [X] T022 [US2] Update /chat endpoint handler in backend/api.py to accept optional selected_text parameter
- [X] T023 [US2] Add selected_text validation logic (max 5000 chars) per data-model.md
- [X] T024 [US2] Pass selected_text to agent.ask() call in /chat handler (agent.py updated to support selected_text parameter)
- [X] T025 [US2] Update response logging to include selected_text context when present

### Tests for User Story 2

- [X] T026 [P] [US2] Add unit tests for selected_text validation (too long, valid) in backend/tests/test_api.py
- [X] T027 [P] [US2] Add integration test for /chat with selected_text in backend/tests/test_integration.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - readers can ask both general and context-specific questions

---

## Phase 5: User Story 3 - Graceful Error Handling (Priority: P1)

**Goal**: Provide clear, helpful feedback when errors occur (no relevant content, service unavailable, invalid input)

**Independent Test**: Submit various invalid/failing requests and verify all return user-friendly error messages without stack traces

### Implementation for User Story 3

- [X] T028 [US3] Enhance validation error handling in backend/api.py to return structured ErrorResponse with code, message, suggestion
- [X] T029 [US3] Add specific error messages for validation failures per data-model.md error cases
- [X] T030 [US3] Implement no-results handling (confidence="none") to return user-friendly message per data-model.md
- [X] T031 [US3] Add rate limiting or request size enforcement for FR-011 (max 2000 chars)
- [X] T032 [US3] Ensure all error responses follow FR-012 (correct HTTP status codes: 400, 500, 503)

### Tests for User Story 3

- [X] T033 [P] [US3] Add integration tests for all error scenarios in backend/tests/test_integration.py (validation errors, agent errors, retrieval errors, no results)
- [X] T034 [P] [US3] Verify error responses contain no stack traces or internal details (SC-003 compliance) in backend/tests/test_integration.py

**Checkpoint**: All error scenarios should now return clear, actionable feedback to users

---

## Phase 6: Optional Features & Enhancements

**Purpose**: Additional features from spec.md that enhance core functionality

### Module Filtering (FR-010)

- [X] T035 [P] Add module_filter parameter support to /chat endpoint in backend/api.py
- [X] T036 Pass module_filter to agent.ask() call
- [X] T037 [P] Add unit tests for module_filter validation in backend/tests/test_api.py

### Top-K Parameter (FR-010)

- [X] T038 [P] Add top_k parameter support to /chat endpoint in backend/api.py (default=5, range 1-100)
- [X] T039 Pass top_k to agent.ask() call
- [X] T040 [P] Add unit tests for top_k validation in backend/tests/test_api.py

---

## Phase 7: Frontend Integration

**Purpose**: Create React ChatWidget component to connect Docusaurus frontend to FastAPI backend

### Frontend API Client

- [X] T041 [P] Create frontend_book/src/components/ChatWidget/ directory
- [X] T042 [P] Create frontend_book/src/components/ChatWidget/api.ts with TypeScript interfaces (ChatRequest, ChatResponse, Citation)
- [X] T043 Implement sendChatQuery() function in api.ts using fetch API per research.md
- [X] T044 Add error handling in api.ts for HTTP error responses

### ChatWidget Component

- [X] T045 [P] Create frontend_book/src/components/ChatWidget/index.tsx with React component structure
- [X] T046 [P] Create frontend_book/src/components/ChatWidget/ChatWidget.module.css for component styling
- [X] T047 Implement query input form in ChatWidget/index.tsx (text input, submit button)
- [X] T048 Add state management for query, loading, response, error in ChatWidget/index.tsx
- [X] T049 Integrate api.ts sendChatQuery() call on form submit in ChatWidget/index.tsx
- [X] T050 Implement response rendering (answer, citations with links, confidence) in ChatWidget/index.tsx
- [X] T051 Implement error message display in ChatWidget/index.tsx
- [X] T052 Add loading state indicator in ChatWidget/index.tsx
- [X] T053 Add timing metrics display (optional) in ChatWidget/index.tsx

### Frontend Configuration

- [X] T054 [P] Create frontend_book/.env.local.example with REACT_APP_API_URL
- [X] T055 [P] ChatWidget integration documentation created (see frontend_book/src/components/ChatWidget/INTEGRATION.md for manual integration steps)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

- [X] T056 [P] Verify OpenAPI schema matches implementation by comparing backend/api.py with specs/007-fastapi-backend-integration/contracts/openapi.yaml
- [X] T057 [P] Add contract tests in backend/tests/test_contract.py to validate OpenAPI schema compliance
- [ ] T058 Run full test suite (pytest backend/tests/) and verify 100% pass rate → See specs/007-fastapi-backend-integration/RUNTIME_VALIDATION.md
- [ ] T059 Test /health endpoint responds within 500ms per SC-008 → See specs/007-fastapi-backend-integration/RUNTIME_VALIDATION.md
- [ ] T060 Test /ready endpoint responds within 500ms per SC-008 → See specs/007-fastapi-backend-integration/RUNTIME_VALIDATION.md
- [X] T061 [P] Update backend/.env.example with production deployment notes
- [X] T062 [P] Create backend/README.md with setup instructions from quickstart.md
- [ ] T063 Manual end-to-end test following quickstart.md validation steps → See specs/007-fastapi-backend-integration/RUNTIME_VALIDATION.md
- [ ] T064 Load testing for SC-002 (50 concurrent requests) using locust or similar tool → See specs/007-fastapi-backend-integration/RUNTIME_VALIDATION.md
- [ ] T065 Verify SC-001 (95% of requests complete within 5 seconds) under normal load → See specs/007-fastapi-backend-integration/RUNTIME_VALIDATION.md
- [X] T066 [P] Code cleanup and remove any debug print statements
- [X] T067 [P] Security review: verify no secrets in code, CORS properly configured, input validation complete

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (Book-Wide Query) - P1: Core functionality
  - User Story 2 (Selected Text) - P2: Enhancement to US1
  - User Story 3 (Error Handling) - P1: Cross-cutting, affects US1 and US2
- **Optional Features (Phase 6)**: Can start after US1 is complete
- **Frontend Integration (Phase 7)**: Can start after US1 backend is complete
- **Polish (Phase 8)**: Depends on all desired user stories + frontend being complete

### User Story Dependencies

- **User Story 1 (P1 - Book-Wide Query)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2 - Selected Text)**: Depends on US1 completion (extends /chat endpoint)
- **User Story 3 (P1 - Error Handling)**: Can start in parallel with US1 (different aspects of same endpoint) but ideally after US1 core implementation

### Within Each User Story

- Implementation tasks before tests (TDD reversed due to existing agent.py - integration approach)
- Core endpoint logic before error handling
- Validation before business logic
- Logging after core functionality

### Parallel Opportunities

**Phase 1 (Setup)**: T002, T003, T004 can run in parallel (different files)

**Phase 2 (Foundational)**: T006, T007, T008, T009 can run in parallel after T005 (all different concerns)

**User Story 1 Tests**: T019, T020, T021 can run in parallel (different test files/suites)

**User Story 2 Tests**: T026, T027 can run in parallel

**User Story 3 Tests**: T033, T034 can run in parallel

**Phase 6 (Optional)**: All tasks in each subsection can run in parallel (module_filter group, top_k group)

**Phase 7 (Frontend)**: T041, T042, T045, T046, T054, T055 can run in parallel (different files)

**Phase 8 (Polish)**: T056, T057, T059, T060, T061, T062, T064, T066, T067 can run in parallel (independent checks)

---

## Parallel Example: User Story 1

```bash
# After T011 (foundation complete), launch User Story 1 tests in parallel:
Task T019: "Create unit tests for ChatRequest validation"
Task T020: "Add integration test for /chat endpoint with valid query"
Task T021: "Add integration test for /chat endpoint error cases"

# Or launch foundational setup tasks in parallel:
Task T006: "Initialize FastAPI application with CORS middleware"
Task T007: "Implement custom exception classes"
Task T008: "Register FastAPI exception handlers"
Task T009: "Implement logging configuration"
```

---

## Implementation Strategy

### MVP First (User Story 1 + US3 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T011) - CRITICAL blocking phase
3. Complete Phase 3: User Story 1 (T012-T021) - Core chat functionality
4. Complete Phase 5: User Story 3 (T028-T034) - Error handling (P1 priority)
5. **STOP and VALIDATE**: Test US1 independently using curl or Swagger UI
6. Optional: Add Phase 7 frontend for UI testing
7. Deploy/demo MVP

**MVP Scope**: Backend API with book-wide queries and graceful error handling (no frontend yet)

### Incremental Delivery

1. **Foundation** (Setup + Foundational) → API server runs, health checks work
2. **MVP** (US1 + US3) → Core chat with error handling → Test with curl → Deploy backend
3. **Enhancement** (US2) → Add selected text queries → Deploy update
4. **Optional Features** (Phase 6) → Module filtering and top_k → Deploy update
5. **Full Integration** (Phase 7) → Add frontend ChatWidget → Full user experience
6. **Production Ready** (Phase 8) → Load testing, security review → Launch

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With multiple developers:

1. Team completes Phase 1 + 2 together (foundation)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T012-T021) + tests
   - **Developer B**: User Story 3 (T028-T034) - can start after US1 core (T012-T018)
   - **Developer C**: Frontend setup (Phase 7 - T041-T055)
3. After US1 + US3 complete:
   - **Developer A**: User Story 2 (T022-T027)
   - **Developer B**: Optional features (Phase 6)
   - **Developer C**: Frontend ChatWidget implementation
4. All converge on Phase 8 (Polish) for final validation

---

## Task Count Summary

- **Phase 1 (Setup)**: 4 tasks
- **Phase 2 (Foundational)**: 7 tasks (BLOCKING)
- **Phase 3 (User Story 1)**: 10 tasks (7 implementation + 3 tests)
- **Phase 4 (User Story 2)**: 6 tasks (4 implementation + 2 tests)
- **Phase 5 (User Story 3)**: 7 tasks (5 implementation + 2 tests)
- **Phase 6 (Optional Features)**: 6 tasks
- **Phase 7 (Frontend Integration)**: 15 tasks
- **Phase 8 (Polish)**: 12 tasks

**Total**: 67 tasks

**MVP Scope** (Phase 1 + 2 + 3 + 5): 28 tasks
**Parallel Opportunities**: 25+ tasks can run in parallel (marked with [P])

---

## Notes

- All tasks follow strict format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- [P] tasks = different files or independent concerns, no sequential dependencies
- [Story] label (US1, US2, US3) maps task to specific user story for traceability
- Tests are included but optional since spec doesn't explicitly request TDD
- Each user story should be independently completable and testable
- Backend uses existing agent.py and retrieve.py modules from previous specs
- Frontend ChatWidget can be added to any Docusaurus page
- Stop at any checkpoint to validate story independently
- CORS configuration critical for frontend-backend communication
- Environment variables must match .env.example for deployment
