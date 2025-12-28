# Research: FastAPI Backend & Frontend Integration

**Feature**: 007-fastapi-backend-integration
**Date**: 2025-12-28
**Phase**: 0 (Outline & Research)

## Research Questions

Based on the Technical Context, the following areas required research to inform design decisions:

### 1. FastAPI Best Practices for Stateless RAG APIs

**Question**: What are the recommended patterns for building stateless FastAPI applications that wrap existing agent/retrieval logic?

**Decision**: Use FastAPI with dependency injection for agent instantiation, Pydantic models for request/response validation, and standard exception handlers for consistent error responses.

**Rationale**:
- FastAPI's automatic OpenAPI documentation generation satisfies FR-001 and SC-007 (JSON compliance)
- Pydantic v2 provides zero-overhead validation for FR-005 (request validation)
- Dependency injection allows clean separation between API layer and agent logic (FR-002 integration requirement)
- Built-in async support enables concurrent request handling (SC-002: 50 concurrent requests)

**Alternatives Considered**:
- **Flask**: Rejected - requires manual OpenAPI schema generation, no built-in async support
- **Django REST Framework**: Rejected - too heavyweight for stateless API, brings unnecessary ORM/admin features
- **Starlette (FastAPI's foundation)**: Rejected - FastAPI adds essential features (automatic docs, validation) with minimal overhead

**Implementation Implications**:
- Import `agent.py::ask()` function directly in FastAPI route handlers
- Use `from fastapi import FastAPI, HTTPException, status` for core framework
- Use `from pydantic import BaseModel, Field, constr` for request/response models
- Use `fastapi.middleware.cors import CORSMiddleware` for FR-007 (CORS support)

---

### 2. CORS Configuration for Docusaurus Frontend

**Question**: How should CORS be configured to allow the Docusaurus frontend (localhost:3000 in dev, deployed domain in prod) to call the FastAPI backend?

**Decision**: Use FastAPI's CORSMiddleware with environment-variable-driven allowed origins list.

**Rationale**:
- Supports both local development (http://localhost:3000) and production (https://yourdomain.com)
- Environment variable `CORS_ORIGINS` allows deployment-specific configuration (FR-015)
- FastAPI's built-in CORS middleware is production-ready and well-tested
- Wildcard (`*`) origins avoided for security, even though no auth is required

**Alternatives Considered**:
- **Nginx/Reverse Proxy CORS**: Rejected - adds deployment complexity, violates single-service constraint
- **API Gateway (AWS/Azure)**: Rejected - not free-tier compatible (Principle V: Public Reproducibility)
- **Wildcard origins**: Rejected - security best practice even for public APIs

**Implementation Implications**:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "http://localhost:3000").split(","),
    allow_credentials=False,  # stateless, no cookies
    allow_methods=["POST", "GET"],
    allow_headers=["Content-Type"],
)
```

---

### 3. Error Handling Strategy

**Question**: How should the API transform agent/retrieval errors into user-friendly responses while maintaining internal debugging capability?

**Decision**: Use custom exception handlers with structured error responses, internal logging, and user-friendly messages.

**Rationale**:
- FR-006 requires graceful error handling without exposing internal details
- SC-003 mandates 100% user-friendly error messages (no stack traces)
- FR-009 requires logging for debugging
- HTTP status codes must follow FR-012 (400 for validation, 500 for server errors, 503 for service unavailable)

**Alternatives Considered**:
- **Generic HTTPException**: Rejected - doesn't differentiate between error types
- **Try-catch in every route**: Rejected - violates DRY principle, hard to maintain consistency
- **Custom error classes without handlers**: Rejected - requires route-level handling, inconsistent

**Implementation Implications**:
- Define custom exception classes: `ValidationError`, `AgentError`, `RetrievalError`
- Register FastAPI exception handlers that log full details internally and return sanitized JSON
- Use Python's structlog or standard logging with JSON formatters for FR-009 compliance

Error response format:
```json
{
  "error": {
    "code": "RETRIEVAL_FAILED",
    "message": "Unable to search the book content. Please try again.",
    "suggestion": "Check your query and retry in a few moments."
  }
}
```

---

### 4. Request Validation Patterns

**Question**: How should the API validate query length (FR-011: max 2000 chars), selected text, and module filters?

**Decision**: Use Pydantic models with Field validators and custom validation methods.

**Rationale**:
- Pydantic's `constr(max_length=2000)` enforces FR-011 declaratively
- Field validators provide clear error messages for SC-009 (100% validation coverage)
- Automatic OpenAPI schema generation documents constraints for frontend developers
- Validation happens before route handler execution, failing fast

**Alternatives Considered**:
- **Manual string length checks**: Rejected - error-prone, no automatic docs
- **Middleware validation**: Rejected - less declarative, harder to test
- **Frontend-only validation**: Rejected - never trust client-side validation

**Implementation Implications**:
```python
from pydantic import BaseModel, Field, constr, field_validator
from typing import Literal, Optional

class ChatRequest(BaseModel):
    query: constr(min_length=1, max_length=2000, strip_whitespace=True)
    selected_text: Optional[str] = None
    module_filter: Optional[Literal["intro", "ros2", "simulation", "isaac", "vla"]] = None
    top_k: int = Field(default=5, ge=1, le=100)

    @field_validator('selected_text')
    def validate_selected_text(cls, v):
        if v and len(v) > 5000:  # reasonable limit for context
            raise ValueError("Selected text too long (max 5000 characters)")
        return v
```

---

### 5. Health Check Endpoint Design

**Question**: What should the health check endpoint (FR-014) verify to ensure system readiness?

**Decision**: Implement two endpoints: `/health` (liveness) and `/ready` (readiness), checking agent, retrieval, and database connectivity.

**Rationale**:
- Kubernetes/Docker best practices distinguish liveness (process alive) from readiness (can serve traffic)
- SC-008 requires health checks respond within 500ms
- Must verify: OpenAI API key validity, Qdrant connectivity, agent module import
- Allows deployment platforms to detect degraded states

**Alternatives Considered**:
- **Single `/health` endpoint**: Rejected - doesn't distinguish startup failures from runtime degradation
- **Deep health checks on every request**: Rejected - adds latency, violates SC-001 (5-second response time)
- **No health checks**: Rejected - violates FR-014

**Implementation Implications**:
```python
@app.get("/health")
async def health():
    """Liveness probe - returns 200 if process is alive"""
    return {"status": "ok"}

@app.get("/ready")
async def readiness():
    """Readiness probe - returns 200 if can serve traffic"""
    checks = {
        "agent": check_agent_module(),
        "retrieval": check_qdrant_connection(),
        "env": check_required_env_vars()
    }
    if all(checks.values()):
        return {"status": "ready", "checks": checks}
    raise HTTPException(status_code=503, detail={"status": "not ready", "checks": checks})
```

---

### 6. Frontend Integration Pattern

**Question**: How should the Docusaurus frontend component integrate with the FastAPI backend?

**Decision**: Create a self-contained React ChatWidget component with an api.ts module for HTTP requests using fetch API.

**Rationale**:
- Docusaurus is React-based, so React component is natural fit
- fetch API is browser-native (no external dependencies like axios)
- Self-contained component can be dropped into any Docusaurus page
- api.ts encapsulates backend URL and request logic for testability

**Alternatives Considered**:
- **Docusaurus plugin**: Rejected - overengineered for a single component
- **iFrame embed**: Rejected - CORS complexity, poor UX
- **WebSockets**: Rejected - stateless requirement (FR-008), streaming not needed for 5-second responses

**Implementation Implications**:
```typescript
// src/components/ChatWidget/api.ts
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export interface ChatRequest {
  query: string;
  selected_text?: string;
  module_filter?: string;
  top_k?: number;
}

export interface ChatResponse {
  answer: string;
  citations: Array<{page_title: string; page_url: string; module_name: string}>;
  confidence: 'high' | 'low' | 'none';
  retrieval_time_ms: number;
  generation_time_ms: number;
  total_time_ms: number;
}

export async function sendChatQuery(request: ChatRequest): Promise<ChatResponse> {
  const response = await fetch(`${API_BASE_URL}/chat`, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error.message);
  }

  return response.json();
}
```

---

### 7. Testing Strategy

**Question**: What testing approach ensures FR-005 (validation), FR-006 (error handling), and SC-009 (100% validation coverage)?

**Decision**: Three-layer testing: unit tests (Pydantic models), integration tests (FastAPI TestClient), and contract tests (OpenAPI schema validation).

**Rationale**:
- Unit tests verify Pydantic validation logic (SC-009 requirement)
- Integration tests verify end-to-end request/response flow including agent integration
- Contract tests ensure frontend/backend stay in sync via OpenAPI schema
- FastAPI's TestClient provides zero-setup testing without running real server

**Alternatives Considered**:
- **Manual testing only**: Rejected - no repeatability, violates Principle VI (Test-Driven Quality)
- **E2E tests only**: Rejected - slow, fragile, doesn't test validation edge cases
- **Frontend tests calling real backend**: Rejected - creates test environment dependencies

**Implementation Implications**:
```python
# tests/test_api.py
from fastapi.testclient import TestClient
from backend.api import app

client = TestClient(app)

def test_chat_endpoint_valid_query():
    response = client.post("/chat", json={"query": "What is ROS 2?"})
    assert response.status_code == 200
    assert "answer" in response.json()
    assert "citations" in response.json()

def test_chat_endpoint_empty_query():
    response = client.post("/chat", json={"query": ""})
    assert response.status_code == 400
    assert "error" in response.json()

def test_chat_endpoint_query_too_long():
    response = client.post("/chat", json={"query": "a" * 2001})
    assert response.status_code == 400
```

---

## Summary

All technical unknowns have been resolved. The plan uses:
- **FastAPI** with Pydantic validation and CORS middleware
- **Dependency injection** for agent integration
- **Custom exception handlers** for user-friendly errors
- **Two health endpoints** (`/health` and `/ready`)
- **React ChatWidget** with fetch-based API client
- **Three-layer testing** (unit, integration, contract)

No further clarifications needed. Proceeding to Phase 1 (Design & Contracts).
