# Implementation Plan: FastAPI Backend & Frontend Integration

**Branch**: `007-fastapi-backend-integration` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-fastapi-backend-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a FastAPI backend service that exposes REST API endpoints for the RAG chatbot, connecting the existing Docusaurus frontend to the RAG agent (Spec 3). The API will accept natural language queries, route them to the agent for processing with retrieved context, and return structured JSON responses with answers, citations, and timing metrics. The system must support both book-wide queries and selected-text context queries, handle errors gracefully, and operate statelessly without authentication.

## Technical Context

**Language/Version**: Python 3.11+ (matching existing backend)
**Primary Dependencies**: FastAPI, Uvicorn, existing agent.py module (OpenAI Agents SDK), existing retrieve.py module (Qdrant)
**Storage**: N/A (stateless API, uses existing Qdrant Cloud vector database via retrieve.py)
**Testing**: pytest (matching existing backend testing framework)
**Target Platform**: Linux/Windows server for local development, deployable to cloud platforms (Vercel, Render, Railway)
**Project Type**: Web application (backend API + frontend integration)
**Performance Goals**: 95% of requests complete within 5 seconds (SC-001), support 50 concurrent requests (SC-002)
**Constraints**: Stateless operation (no session management), CORS-enabled for Docusaurus frontend, maximum query length 2000 characters (FR-011)
**Scale/Scope**: Single API service with 3 primary endpoints (chat, health, readiness), integrates with existing Docusaurus UI in frontend_book/

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Spec-Driven Reproducibility ✅
- **Status**: PASS
- **Evidence**: Feature begins with complete specification (spec.md) before implementation
- **Compliance**: Plan follows Spec-Kit Plus workflow (/sp.specify → /sp.plan → /sp.tasks → /sp.implement)

### Principle II: Content Accuracy ✅
- **Status**: PASS
- **Evidence**: API integrates with existing RAG agent (agent.py) which enforces citation-only responses
- **Compliance**: FR-002 mandates integration with Spec 3 agent, FR-004 requires citations in all responses

### Principle III: Developer-Focused Writing ✅
- **Status**: PASS
- **Evidence**: API documentation will use OpenAPI schema for clear endpoint definitions
- **Compliance**: FR-004 specifies structured JSON responses, enabling auto-generated API docs

### Principle IV: Retrieval Transparency ✅
- **Status**: PASS
- **Evidence**: API passes through citations from agent responses (FR-004)
- **Compliance**: ChatResponse entity includes citations list with page title, URL, and module

### Principle V: Public Reproducibility ✅
- **Status**: PASS
- **Evidence**: Uses free-tier infrastructure (Qdrant Cloud, no auth required for local dev)
- **Compliance**: FR-015 mandates environment variable configuration for zero-cost deployment

### Principle VI: Test-Driven Quality ✅
- **Status**: PASS
- **Evidence**: All requirements have testable acceptance criteria in spec.md
- **Compliance**: SC-009 requires 100% validation coverage, testing strategy defined in Phase 1

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── api.py                  # NEW: FastAPI application (this feature)
├── agent.py               # EXISTING: RAG agent from Spec 3
├── retrieve.py            # EXISTING: Retrieval pipeline from Spec 2
├── pyproject.toml         # EXISTING: Python dependencies
├── .env.example           # EXISTING: Environment template
└── tests/                 # NEW: API tests (this feature)
    ├── test_api.py        # Unit tests for FastAPI endpoints
    ├── test_integration.py # End-to-end API integration tests
    └── conftest.py        # Pytest fixtures

frontend_book/
├── src/
│   ├── components/
│   │   └── ChatWidget/    # NEW: Chat UI component (this feature)
│   │       ├── index.tsx
│   │       ├── ChatWidget.module.css
│   │       └── api.ts     # NEW: API client for backend
│   ├── pages/             # EXISTING: Docusaurus pages
│   └── theme/             # EXISTING: Docusaurus theme
├── docs/                  # EXISTING: Book content
├── docusaurus.config.ts   # EXISTING: Docusaurus configuration
└── package.json           # EXISTING: Frontend dependencies
```

**Structure Decision**: Extends existing web application structure. The backend/ directory already contains agent.py and retrieve.py from previous specs. This feature adds api.py at the same level to create a FastAPI server that imports and uses the existing agent. The frontend_book/ directory already contains the Docusaurus site; we add a new ChatWidget component that makes HTTP requests to the backend API. This minimal structure avoids creating new directories and leverages existing code.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - No constitution violations. All principles pass without requiring complexity justification.
