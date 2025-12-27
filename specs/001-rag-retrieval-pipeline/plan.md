# Implementation Plan: RAG Retrieval Pipeline

**Branch**: `001-rag-retrieval-pipeline` | **Date**: 2025-12-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-retrieval-pipeline/spec.md`

## Summary

Build a retrieval layer (`retrieve.py`) that queries the Qdrant vector database populated by Spec 1, generates query embeddings using Cohere, performs similarity search with optional filtering, and validates retrieval accuracy across all book modules. This is a single-file CLI tool for testing the RAG pipeline.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client, python-dotenv (same as Spec 1)
**Storage**: Qdrant Cloud (read-only - vectors from Spec 1)
**Testing**: Manual validation via CLI + automated test suite
**Target Platform**: Local CLI execution (Windows/Linux/macOS)
**Project Type**: Single backend project (extends existing `backend/`)
**Performance Goals**: Query-to-results in <2 seconds
**Constraints**: Free tier APIs, reuse Spec 1 configuration
**Scale/Scope**: 288 indexed vectors, 5 modules, 5+ test queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-Driven Reproducibility | PASS | Spec created first, plan follows spec |
| II. Content Accuracy | PASS | Retrieval only - no generation/hallucination risk |
| III. Developer-Focused Writing | N/A | No user-facing content in this spec |
| IV. Retrieval Transparency | PASS | Returns source URLs and similarity scores |
| V. Public Reproducibility | PASS | Uses free tier services (Qdrant Cloud, Cohere) |
| VI. Test-Driven Quality | PASS | Validation test suite with 80% accuracy threshold |

**Technology Standards Alignment**:
- Vector Database: Qdrant Cloud (Free Tier) ✅
- API Layer: Python CLI (FastAPI not needed for this spec) ✅
- Secrets: Environment variables via .env ✅

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-retrieval-pipeline/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technical decisions
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
├── .env                 # API keys (not committed) - from Spec 1
├── .env.example         # Template for .env - from Spec 1
├── .gitignore           # Ignore .env, __pycache__, .venv - from Spec 1
├── pyproject.toml       # UV project configuration - from Spec 1
├── uv.lock              # Locked dependencies - from Spec 1
├── main.py              # Ingestion pipeline (Spec 1)
└── retrieve.py          # Retrieval pipeline (THIS SPEC) - NEW FILE
```

**Structure Decision**: Single new file `retrieve.py` in existing `backend/` directory. Reuses all configuration and dependencies from Spec 1. No new packages required.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                     retrieve.py (Single File)                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │ Query Input  │───▶│  Embedding   │───▶│   Qdrant Search      │  │
│  │              │    │              │    │                      │  │
│  │ - CLI args   │    │ - Cohere API │    │ - Similarity search  │  │
│  │ - Filters    │    │ - search_query│   │ - Payload filters    │  │
│  │ - top-k      │    │ - 1024 dims  │    │ - Top-k ranking      │  │
│  └──────────────┘    └──────────────┘    └──────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                     Output Formatting                         │  │
│  │  - Standard text output                                      │  │
│  │  - Verbose mode with chunk text                              │  │
│  │  - JSON output for programmatic use                          │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                     Validation Mode                           │  │
│  │  - Predefined test queries per module                        │  │
│  │  - Expected module matching                                  │  │
│  │  - Pass/fail reporting with accuracy %                       │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

External Services (Shared with Spec 1):
┌─────────────────┐      ┌─────────────────┐
│   Cohere API    │      │  Qdrant Cloud   │
│  (Query Embed)  │      │  (Vector Search)│
└─────────────────┘      └─────────────────┘
```

## Implementation Phases

### Phase 1: Core Retrieval
- Load config from .env (reuse Spec 1 pattern)
- Connect to Qdrant collection
- Implement query embedding with Cohere (`input_type="search_query"`)
- Implement similarity search with top-k
- Format and display results

### Phase 2: Filtering
- Add module filter support
- Add URL filter support
- Combine filters with AND logic

### Phase 3: CLI Interface
- Parse command line arguments
- Implement `--top-k`, `--module`, `--url` flags
- Implement `--verbose` and `--json` output modes
- Handle errors with appropriate exit codes

### Phase 4: Validation Suite
- Define test queries with expected modules
- Implement `--validate` mode
- Calculate accuracy metrics
- Generate validation report

## Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Single file | `retrieve.py` only | User requested simplicity; mirrors Spec 1 structure |
| Query embedding | `input_type="search_query"` | Cohere best practice for asymmetric search |
| Filtering | Qdrant native filters | Efficient, happens during search |
| Output modes | Text/Verbose/JSON | Flexibility for testing and integration |
| Validation | Embedded test queries | Simple, deterministic, no external files |

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Cohere rate limits | Single query at a time, retry with backoff |
| Low relevance scores | Return scores for user interpretation |
| Module mismatch | 80% accuracy threshold, not 100% |
| Network failures | Clear error messages, fail fast |

## Complexity Tracking

No constitution violations to justify. Single-file structure is the simplest option.

## Related Artifacts

- [research.md](./research.md) - Technology research and decisions
- [data-model.md](./data-model.md) - Entity definitions
- [quickstart.md](./quickstart.md) - Setup and usage instructions
- [contracts/cli-interface.md](./contracts/cli-interface.md) - CLI behavior contract

## Next Steps

Run `/sp.tasks` to generate the task breakdown for implementation.
