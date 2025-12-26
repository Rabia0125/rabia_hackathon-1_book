# Implementation Plan: RAG Data Pipeline

**Branch**: `001-rag-data-pipeline` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-data-pipeline/spec.md`

## Summary

Build a Python CLI pipeline that scrapes content from the Physical AI & Robotics Docusaurus book site (https://rabia-hackathon-1-book.vercel.app), generates vector embeddings using Cohere API, and stores them in Qdrant Cloud for RAG retrieval. The pipeline must be idempotent (re-runnable without duplicates) and provide validation reporting.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client, httpx, beautifulsoup4, python-dotenv
**Storage**: Qdrant Cloud Free Tier (vector database)
**Testing**: Manual validation (pipeline output verification)
**Target Platform**: Local CLI execution (Windows/Linux/macOS)
**Project Type**: Single backend project
**Performance Goals**: Complete full indexing in <10 minutes for ~22 content pages
**Constraints**: Free tier APIs, polite crawling (0.5s delay), batch operations
**Scale/Scope**: ~77 sitemap URLs, ~22 content pages, estimated ~100-150 chunks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-Driven Reproducibility | PASS | Spec created first, plan follows spec |
| II. Content Accuracy | N/A | This is ingestion, not retrieval (Spec 2+ concern) |
| III. Developer-Focused Writing | N/A | No user-facing content in this spec |
| IV. Retrieval Transparency | N/A | Deferred to Spec 2 |
| V. Public Reproducibility | PASS | Uses free tier services (Qdrant Cloud, Cohere) |
| VI. Test-Driven Quality | PASS | Acceptance criteria defined, validation reporting included |

**Technology Standards Alignment**:
- Vector Database: Qdrant Cloud (Free Tier) ✅
- API Layer: Python (FastAPI not needed for CLI) ✅
- Secrets: Environment variables via .env ✅

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-data-pipeline/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup and usage guide
├── contracts/           # Interface contracts
│   └── cli-interface.md # CLI behavior contract
├── checklists/          # Validation checklists
│   └── requirements.md  # Spec quality checklist
└── tasks.md             # Task breakdown (created by /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── .env                 # API keys (not committed)
├── .env.example         # Template for .env
├── .gitignore           # Ignore .env, __pycache__, .venv
├── pyproject.toml       # UV project configuration
├── uv.lock              # Locked dependencies
└── main.py              # Pipeline entry point (single file per user request)
```

**Structure Decision**: Single-file backend (`main.py`) per user's execution plan. All logic consolidated in one file for simplicity. Future specs may refactor into modules if needed.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        main.py (Single File)                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │   Scraper    │───▶│   Chunker    │───▶│  Embedding + Store   │  │
│  │              │    │              │    │                      │  │
│  │ - sitemap.xml│    │ - 800 chars  │    │ - Cohere embed API   │  │
│  │ - httpx GET  │    │ - 100 overlap│    │ - Qdrant upsert      │  │
│  │ - BS4 parse  │    │ - metadata   │    │ - Batch processing   │  │
│  └──────────────┘    └──────────────┘    └──────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                     main() Entry Point                        │  │
│  │  1. Validate env vars                                        │  │
│  │  2. Fetch sitemap and filter content URLs                    │  │
│  │  3. Scrape pages sequentially (0.5s delay)                   │  │
│  │  4. Chunk text with overlap                                  │  │
│  │  5. Generate embeddings in batches (96 per call)             │  │
│  │  6. Upsert vectors to Qdrant (100 per batch)                 │  │
│  │  7. Print summary report                                     │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

External Services:
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│  Docusaurus     │      │   Cohere API    │      │  Qdrant Cloud   │
│  (Source)       │      │  (Embeddings)   │      │  (Vector DB)    │
└─────────────────┘      └─────────────────┘      └─────────────────┘
```

## Implementation Phases

### Phase 1: Project Setup
- Create `backend/` directory
- Initialize UV project with `uv init`
- Add dependencies
- Create `.env.example` template
- Add `.gitignore`

### Phase 2: Core Pipeline Logic
In `main.py`:
1. **Config & Validation**: Load env vars, validate required keys
2. **Sitemap Parsing**: Fetch and parse sitemap.xml, filter content URLs
3. **Page Scraping**: HTTP GET with retry, HTML to text extraction
4. **Text Chunking**: Character-based with overlap, preserve metadata
5. **Embedding Generation**: Cohere batch API calls
6. **Vector Storage**: Qdrant upsert with deterministic IDs

### Phase 3: Reporting & Validation
- Progress output during processing
- Summary statistics on completion
- Module breakdown report
- Error logging

## Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Single file | `main.py` only | User requested simplicity; can refactor later |
| Package manager | UV | Fast, modern, handles Python version |
| HTTP client | httpx | Async-capable, modern API |
| HTML parser | BeautifulSoup4 | Simple, reliable for static HTML |
| Embeddings | Cohere embed-v3-english | Good quality, free tier available |
| Vector DB | Qdrant Cloud | Free tier, good Python client |
| Deduplication | UUID5 from URL+index | Deterministic, enables upsert |
| Chunk strategy | 800 chars, 100 overlap | Balances context and retrieval precision |

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Cohere rate limits | Batch requests (96 per call), exponential backoff |
| Qdrant free tier limits | Monitor storage usage, ~100 chunks well within 1GB |
| Site structure changes | Sitemap-based discovery, not hardcoded URLs |
| Network failures | Retry with backoff, continue on single page failure |

## Complexity Tracking

No constitution violations to justify. Single-project structure is the simplest option.

## Related Artifacts

- [research.md](./research.md) - Technology research and decisions
- [data-model.md](./data-model.md) - Entity definitions and Qdrant schema
- [quickstart.md](./quickstart.md) - Setup and usage instructions
- [contracts/cli-interface.md](./contracts/cli-interface.md) - CLI behavior contract

## Next Steps

Run `/sp.tasks` to generate the task breakdown for implementation.
