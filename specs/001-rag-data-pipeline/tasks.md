# Tasks: RAG Data Pipeline

**Input**: Design documents from `/specs/001-rag-data-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not requested - manual validation via pipeline output

**Organization**: Tasks grouped by user story for independent implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1, US2, US3 (maps to spec.md user stories)
- All paths relative to repository root

---

## Phase 1: Setup

**Purpose**: Initialize UV project and configure environment

- [x] T001 Create `backend/` directory and initialize UV project with `uv init backend`
- [x] T002 Add dependencies: `uv add cohere qdrant-client python-dotenv httpx beautifulsoup4`
- [x] T003 [P] Create `backend/.env.example` with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, TARGET_URL, COLLECTION_NAME
- [x] T004 [P] Create `backend/.gitignore` to exclude .env, __pycache__, .venv

---

## Phase 2: Foundational

**Purpose**: Core config validation in main.py (blocks all user stories)

- [x] T005 Create `backend/main.py` with imports and load_config() function using python-dotenv
- [x] T006 Add validate_config() to fail fast with clear error if required env vars missing

**Checkpoint**: Config loading works, validates .env

---

## Phase 3: User Story 1 - Initial Content Ingestion (P1) MVP

**Goal**: Scrape all book content, embed, and store in Qdrant

**Independent Test**: Run `uv run python main.py` and verify vectors in Qdrant Cloud dashboard

### Implementation

- [x] T007 [US1] Add fetch_sitemap(url) in main.py - parse sitemap.xml, return list of content URLs (filter out /tags/)
- [x] T008 [US1] Add extract_module(url) in main.py - derive module name from URL path (/docs/module-1/* → ros2)
- [x] T009 [US1] Add scrape_page(url) in main.py - httpx GET with 0.5s delay, BeautifulSoup text extraction, return Page dict
- [x] T010 [US1] Add chunk_text(text, size=800, overlap=100) in main.py - split text, return list of Chunk dicts
- [x] T011 [US1] Add generate_chunk_id(url, index) in main.py - UUID5 from URL::index
- [x] T012 [US1] Add generate_embeddings(chunks) in main.py - Cohere batch API (96 per call), return embeddings list
- [x] T013 [US1] Add init_qdrant_collection(client, name) in main.py - create collection if not exists, 1024 dims, Cosine
- [x] T014 [US1] Add upsert_vectors(client, collection, chunks, embeddings) in main.py - PointStruct with payload metadata
- [x] T015 [US1] Add main() in main.py - orchestrate: load config → fetch sitemap → scrape pages → chunk → embed → upsert

**Checkpoint**: Full ingestion pipeline works end-to-end

---

## Phase 4: User Story 2 - Content Update/Re-indexing (P2)

**Goal**: Re-run pipeline without duplicates, clean stale vectors

**Independent Test**: Re-run pipeline, verify vector count unchanged, no duplicates

### Implementation

- [x] T016 [US2] Modify upsert_vectors() to use deterministic IDs (already UUID5, verify upsert behavior)
- [x] T017 [US2] Add cleanup_stale_vectors(client, collection, current_ids) in main.py - delete vectors not in current batch
- [x] T018 [US2] Update main() to track all chunk IDs and call cleanup after upsert

**Checkpoint**: Re-indexing replaces old content, removes deleted pages

---

## Phase 5: User Story 3 - Validation & Reporting (P3)

**Goal**: Report statistics and validate module coverage

**Independent Test**: Run pipeline, verify summary output shows pages/chunks/vectors per module

### Implementation

- [x] T019 [US3] Add PipelineStats class in main.py - track pages_scraped, chunks_created, vectors_stored, errors, by_module dict
- [x] T020 [US3] Update main() to collect stats during processing
- [x] T021 [US3] Add print_summary(stats) in main.py - formatted output with totals and per-module breakdown
- [x] T022 [US3] Add error handling in scrape_page() - log URL and error, continue processing

**Checkpoint**: Pipeline outputs comprehensive validation report

---

## Phase 6: Polish

**Purpose**: Error handling, edge cases, documentation

- [x] T023 [P] Add retry with exponential backoff to scrape_page() for network errors (3 attempts)
- [x] T024 [P] Add rate limit handling to generate_embeddings() with sleep and retry
- [x] T025 Validate pipeline with `uv run python main.py` and verify Qdrant dashboard

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies
- **Phase 2 (Foundational)**: Depends on Phase 1
- **Phase 3 (US1)**: Depends on Phase 2 - core pipeline
- **Phase 4 (US2)**: Depends on Phase 3 - extends upsert
- **Phase 5 (US3)**: Depends on Phase 3 - adds reporting
- **Phase 6 (Polish)**: Depends on Phase 3

### Parallel Opportunities

```bash
# Phase 1 parallel:
T003, T004 (different files)

# Phase 3 parallel (different functions, same file - execute sequentially but logically independent):
T007, T008 (URL handling functions)
T009, T010, T011 (scraping/chunking functions)
T012, T013, T014 (embedding/storage functions)

# Phase 6 parallel:
T023, T024 (independent error handling)
```

---

## Implementation Strategy

### MVP (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T006)
3. Complete Phase 3: User Story 1 (T007-T015)
4. **VALIDATE**: Run pipeline, check Qdrant dashboard
5. **MVP DONE**: Basic ingestion working

### Full Implementation

1. MVP complete
2. Add Phase 4: US2 (T016-T018) - re-indexing
3. Add Phase 5: US3 (T019-T022) - reporting
4. Add Phase 6: Polish (T023-T025) - robustness

---

## Task Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Setup | T001-T004 | Project init |
| Foundational | T005-T006 | Config |
| US1 (P1) | T007-T015 | Core pipeline |
| US2 (P2) | T016-T018 | Re-indexing |
| US3 (P3) | T019-T022 | Reporting |
| Polish | T023-T025 | Robustness |

**Total: 25 tasks** | **MVP: 15 tasks (T001-T015)**
