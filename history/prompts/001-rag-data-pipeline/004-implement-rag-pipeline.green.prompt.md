---
id: 004
title: Implement RAG Data Pipeline
stage: green
date: 2025-12-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-data-pipeline
branch: 001-rag-data-pipeline
user: developer
command: /sp.implement
labels: ["implementation", "python", "cohere", "qdrant", "web-scraping", "embeddings"]
links:
  spec: specs/001-rag-data-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/pyproject.toml
  - backend/main.py
  - backend/.env.example
  - backend/.gitignore
  - specs/001-rag-data-pipeline/tasks.md
tests:
  - Pipeline execution test (ran successfully)
  - Cohere embedding generation (1 chunk embedded)
  - Qdrant vector storage (1 vector stored)
---

## Prompt

/sp.implement

## Response snapshot

Implemented complete RAG Data Pipeline in single `backend/main.py` file:

**Phase 1: Setup (T001-T004)**
- Created `backend/` directory with pyproject.toml
- Added dependencies: cohere, qdrant-client, httpx, beautifulsoup4, python-dotenv
- Created .env.example and .gitignore

**Phase 2: Foundational (T005-T006)**
- Config dataclass with load_config() and validate_config()
- Environment variable loading via python-dotenv

**Phase 3: User Story 1 - Initial Ingestion (T007-T015)**
- fetch_sitemap() - parses sitemap.xml, filters content URLs
- extract_module() - derives module from URL path
- scrape_page() - httpx GET with retry, BeautifulSoup extraction
- chunk_text() - character-based with overlap
- generate_chunk_id() - UUID5 for deduplication
- generate_embeddings() - Cohere batch API
- init_qdrant_collection() - creates collection with 1024 dims
- upsert_vectors() - batch upsert with metadata payload
- main() - orchestrates full pipeline

**Phase 4: User Story 2 - Re-indexing (T016-T018)**
- Deterministic UUID5 IDs enable upsert behavior
- cleanup_stale_vectors() removes deleted content

**Phase 5: User Story 3 - Reporting (T019-T022)**
- PipelineStats class tracks all metrics
- print_summary() with module breakdown
- Error logging continues processing

**Phase 6: Polish (T023-T025)**
- Exponential backoff retry (3 attempts)
- Cohere rate limit handling with 60s wait

**Validation Run**:
- Pipeline executed successfully
- 1 page scraped (homepage), 1 vector stored
- 19 pages returned 404 (Vercel deployment issue with sitemap)
- Collection created in Qdrant Cloud

## Outcome

- Impact: Complete RAG data pipeline ready for use; site deployment needs fixing for full ingestion
- Tests: Pipeline ran, Cohere API connected, Qdrant collection created
- Files: 5 files created/modified
- Next prompts: Fix Vercel deployment to match sitemap URLs, then re-run pipeline
- Reflection: Pipeline code is correct; 404 errors indicate stale sitemap vs actual deployed content

## Evaluation notes (flywheel)

- Failure modes observed: Target site sitemap references non-existent pages (deployment mismatch)
- Graders run and results (PASS/FAIL): Code execution PASS, full ingestion PARTIAL (site issue)
- Prompt variant (if applicable): Standard /sp.implement
- Next experiment (smallest change to try): Rebuild and redeploy Docusaurus site to fix 404s
