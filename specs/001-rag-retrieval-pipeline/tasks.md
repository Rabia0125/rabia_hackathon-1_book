# Tasks: RAG Retrieval Pipeline

**Input**: Design documents from `/specs/001-rag-retrieval-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not requested - manual validation via CLI and `--validate` mode

**Organization**: Tasks grouped by user story for independent implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1, US2, US3 (maps to spec.md user stories)
- All paths relative to repository root

---

## Phase 1: Setup

**Purpose**: Create retrieve.py file and set up basic structure

- [x] T001 Create `backend/retrieve.py` with docstring, imports (cohere, qdrant_client, dotenv, argparse, sys, time, json, dataclasses)
- [x] T002 Add Config dataclass in `backend/retrieve.py` with cohere_api_key, qdrant_url, qdrant_api_key, collection_name
- [x] T003 Add load_config() function in `backend/retrieve.py` using python-dotenv (reuse pattern from main.py)
- [x] T004 Add validate_config() function in `backend/retrieve.py` to fail fast with clear errors if env vars missing

**Checkpoint**: Config loading works, validates .env

---

## Phase 2: Foundational

**Purpose**: Core data classes needed by all user stories

- [x] T005 Add SearchResult dataclass in `backend/retrieve.py` with text, score, page_title, page_url, module_name, chunk_index
- [x] T006 Add RetrievalResponse dataclass in `backend/retrieve.py` with query, results, total_found, query_time_ms, filters_applied

**Checkpoint**: Data models ready for retrieval functions

---

## Phase 3: User Story 1 - Basic Query Retrieval (P1) MVP

**Goal**: Submit queries and receive ranked results with similarity scores

**Independent Test**: Run `uv run python retrieve.py "What is ROS 2?"` and verify results from ros2 module

### Implementation

- [x] T007 [US1] Add embed_query(text, cohere_client) in `backend/retrieve.py` - Cohere embed with input_type="search_query", return 1024-dim vector
- [x] T008 [US1] Add search_qdrant(query_vector, client, collection, top_k) in `backend/retrieve.py` - QdrantClient.search() with with_payload=True
- [x] T009 [US1] Add format_results(qdrant_results) in `backend/retrieve.py` - convert Qdrant ScoredPoints to list of SearchResult
- [x] T010 [US1] Add retrieve(query, config, top_k=5) in `backend/retrieve.py` - orchestrate: embed → search → format, return RetrievalResponse
- [x] T011 [US1] Add print_results(response, verbose=False) in `backend/retrieve.py` - format output per CLI contract (standard mode)
- [x] T012 [US1] Add basic main() in `backend/retrieve.py` - parse positional query arg, call retrieve(), print results

**Checkpoint**: Basic query retrieval works end-to-end

---

## Phase 4: User Story 2 - Filtered Retrieval (P2)

**Goal**: Filter results by module or URL

**Independent Test**: Run `uv run python retrieve.py "sensors" --module simulation` and verify only simulation module results

### Implementation

- [x] T013 [US2] Add build_filter(module=None, url=None) in `backend/retrieve.py` - construct Qdrant Filter with FieldCondition
- [x] T014 [US2] Modify search_qdrant() to accept optional filter parameter in `backend/retrieve.py`
- [x] T015 [US2] Modify retrieve() to accept module_filter and url_filter, pass to search in `backend/retrieve.py`
- [x] T016 [US2] Add --module and --url CLI flags in main() in `backend/retrieve.py`
- [x] T017 [US2] Update print_results() to show applied filters in `backend/retrieve.py`

**Checkpoint**: Filtered retrieval works with module and URL filters

---

## Phase 5: User Story 3 - Validation & Test Suite (P3)

**Goal**: Run test queries covering all modules and report accuracy

**Independent Test**: Run `uv run python retrieve.py --validate` and verify 80%+ accuracy

### Implementation

- [x] T018 [US3] Add TEST_QUERIES constant in `backend/retrieve.py` - list of (query, expected_module) tuples for all 5 modules
- [x] T019 [US3] Add ValidationResult dataclass in `backend/retrieve.py` with query, expected_module, actual_module, top_score, passed
- [x] T020 [US3] Add ValidationReport dataclass in `backend/retrieve.py` with total_tests, passed, failed, accuracy, results, execution_time_ms
- [x] T021 [US3] Add run_validation(config) in `backend/retrieve.py` - execute all test queries, build ValidationReport
- [x] T022 [US3] Add print_validation_report(report) in `backend/retrieve.py` - formatted output per CLI contract
- [x] T023 [US3] Add --validate flag in main() in `backend/retrieve.py` - run validation instead of query

**Checkpoint**: Validation suite runs and reports accuracy metrics

---

## Phase 6: Polish

**Purpose**: Additional CLI features, error handling, output modes

- [x] T024 [P] Add --top-k flag parsing and validation (1-100) in main() in `backend/retrieve.py`
- [x] T025 [P] Add --verbose flag to show chunk text in print_results() in `backend/retrieve.py`
- [x] T026 [P] Add --json flag to output JSON format in `backend/retrieve.py`
- [x] T027 [P] Add print_json(response) in `backend/retrieve.py` - JSON output per CLI contract
- [x] T028 Add error handling for empty query in main() in `backend/retrieve.py`
- [x] T029 Add error handling for Cohere API failures with retry in embed_query() in `backend/retrieve.py`
- [x] T030 Add error handling for Qdrant connection failures in search_qdrant() in `backend/retrieve.py`
- [x] T031 Add appropriate exit codes per CLI contract in main() in `backend/retrieve.py`
- [x] T032 Validate pipeline with `uv run python retrieve.py --validate` and verify 80%+ accuracy

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies
- **Phase 2 (Foundational)**: Depends on Phase 1
- **Phase 3 (US1)**: Depends on Phase 2 - core retrieval MVP
- **Phase 4 (US2)**: Depends on Phase 3 - adds filtering
- **Phase 5 (US3)**: Depends on Phase 3 - adds validation
- **Phase 6 (Polish)**: Depends on Phase 3

### User Story Independence

```text
US1 (P1): Basic retrieval → MVP, can be tested alone
US2 (P2): Filtering → depends on US1, adds filter capability
US3 (P3): Validation → depends on US1, adds test suite
```

### Parallel Opportunities

```bash
# Phase 6 parallel tasks (different functions, no dependencies):
T024, T025, T026, T027 (CLI flag handling)
T029, T030 (error handling in different functions)
```

---

## Implementation Strategy

### MVP (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T006)
3. Complete Phase 3: User Story 1 (T007-T012)
4. **VALIDATE**: Run `uv run python retrieve.py "What is ROS 2?"` and check results
5. **MVP DONE**: Basic retrieval working

### Full Implementation

1. MVP complete
2. Add Phase 4: US2 (T013-T017) - filtering
3. Add Phase 5: US3 (T018-T023) - validation
4. Add Phase 6: Polish (T024-T032) - CLI features, error handling
5. **FINAL VALIDATION**: Run `uv run python retrieve.py --validate`

---

## Task Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Setup | T001-T004 | Project init |
| Foundational | T005-T006 | Data models |
| US1 (P1) | T007-T012 | Core retrieval |
| US2 (P2) | T013-T017 | Filtering |
| US3 (P3) | T018-T023 | Validation |
| Polish | T024-T032 | CLI & errors |

**Total: 32 tasks** | **MVP: 12 tasks (T001-T012)**
