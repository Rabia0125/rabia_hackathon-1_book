---
id: 008
title: FastAPI Backend Integration Plan
stage: plan
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5
feature: 007-fastapi-backend-integration
branch: 007-fastapi-backend-integration
user: DELL LATITUDE
command: /sp.plan
labels: ["rag-chatbot", "fastapi", "backend", "api", "plan", "architecture"]
links:
  spec: specs/007-fastapi-backend-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-fastapi-backend-integration/plan.md
  - specs/007-fastapi-backend-integration/research.md
  - specs/007-fastapi-backend-integration/data-model.md
  - specs/007-fastapi-backend-integration/contracts/openapi.yaml
  - specs/007-fastapi-backend-integration/quickstart.md
  - CLAUDE.md
  - history/prompts/007-fastapi-backend-integration/008-fastapi-backend-integration-plan.plan.prompt.md
tests:
  - Constitution Check validated - all 6 principles pass
  - Research resolved 7 technical unknowns
  - Data model defined 5 entities with validation rules
  - OpenAPI contract created with 3 endpoints
---

## Prompt

RAG Chatbot – Spec 4 Execution Plan

- Use existing Docusaurus UI in frontend_book/ and add Chatbot UI components
- Connect frontend chat UI to backend via HTTP requests
- Create api.py at project root with FastAPI server setup
- In api.py, import and invoke agent from agent.py to generate responses
- Validate end-to-end flow: frontend → FastAPI → agent → response

## Response snapshot

Created comprehensive implementation plan for FastAPI Backend & Frontend Integration (Spec 4):

**Branch**: `007-fastapi-backend-integration`

### Summary
FastAPI backend service exposing REST API endpoints that connect the existing Docusaurus frontend to the RAG agent (Spec 3). Supports natural language queries, optional selected-text context, module filtering, and stateless operation with comprehensive error handling.

### Technical Context
- **Language**: Python 3.11+ (matching existing backend)
- **Dependencies**: FastAPI, Uvicorn, existing agent.py (OpenAI Agents SDK), existing retrieve.py (Qdrant)
- **Testing**: pytest with 3-layer strategy (unit, integration, contract)
- **Performance Goals**: 95% requests within 5 seconds, 50 concurrent requests support
- **Constraints**: Stateless, CORS-enabled, max query 2000 chars

### Constitution Check ✅
All 6 principles PASS:
- **Spec-Driven Reproducibility**: Complete spec before implementation
- **Content Accuracy**: Integrates with existing citation-enforcing agent
- **Developer-Focused Writing**: OpenAPI auto-generated documentation
- **Retrieval Transparency**: Citations passed through in all responses
- **Public Reproducibility**: Free-tier infrastructure, env-based config
- **Test-Driven Quality**: Comprehensive testing strategy defined

### Project Structure
Extends existing web application:
- `backend/api.py` - NEW FastAPI application
- `backend/tests/` - NEW API test suite
- `frontend_book/src/components/ChatWidget/` - NEW React chat component

### Phase 0: Research (research.md)
Resolved 7 technical decisions:
1. FastAPI with Pydantic validation chosen over Flask/Django
2. CORS configuration via CORSMiddleware with env-driven origins
3. Custom exception handlers for user-friendly error responses
4. Pydantic Field validators for request validation (max 2000 chars)
5. Two health endpoints: `/health` (liveness), `/ready` (readiness)
6. React ChatWidget with fetch-based API client
7. Three-layer testing: unit, integration, contract

### Phase 1: Design & Contracts
**data-model.md**: 5 entities defined
- ChatRequest: query, selected_text, module_filter, top_k (with validation rules)
- ChatResponse: answer, citations, confidence, timing metrics
- Citation: page_title, page_url, module_name
- ErrorDetail: code, message, suggestion
- HealthStatus: status, checks, version

**contracts/openapi.yaml**: OpenAPI 3.1 contract with 3 endpoints
- `POST /chat`: Main query endpoint with examples for high/low/none confidence
- `GET /health`: Liveness probe
- `GET /ready`: Readiness probe with component checks

**quickstart.md**: Developer guide covering:
- Backend setup (FastAPI + Uvicorn)
- Frontend setup (Docusaurus integration)
- End-to-end testing workflow
- Common issues and solutions
- Deployment options (Vercel, Render, Railway)

### Agent Context Update
Updated `CLAUDE.md` with new technologies:
- Python 3.11+ (matching existing backend)
- FastAPI, Uvicorn
- Stateless API pattern

## Outcome

- ✅ Impact: Complete architectural plan for FastAPI backend integration ready for task breakdown
- 🧪 Tests: Constitution check passed, research validated, data model and contracts defined
- 📁 Files: plan.md, research.md, data-model.md, openapi.yaml, quickstart.md, CLAUDE.md updated
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks, followed by `/sp.implement`
- 🧠 Reflection: Plan successfully balances technical detail with implementation flexibility. Research phase resolved all unknowns without requiring clarifications. OpenAPI contract enables frontend/backend teams to work in parallel.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully on first iteration
- Graders run and results (PASS/FAIL): Constitution Check PASS (6/6 principles), Technical Context complete, Project Structure validated
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (Research) + Phase 1 (Design & Contracts)
- Next experiment (smallest change to try): Proceed with /sp.tasks to break down plan into concrete, testable implementation tasks following TDD principles
