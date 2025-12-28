# Data Model: FastAPI Backend & Frontend Integration

**Feature**: 007-fastapi-backend-integration
**Date**: 2025-12-28
**Phase**: 1 (Design & Contracts)

## Overview

This feature defines request/response data structures for the FastAPI backend. All entities are implemented as Pydantic models for automatic validation and OpenAPI schema generation. The system is stateless (FR-008), so no persistent data models or database schemas are required.

---

## Entities

### 1. ChatRequest

**Purpose**: Represents an incoming query from the Docusaurus frontend to the `/chat` endpoint.

**Fields**:

| Field           | Type                                                 | Required | Constraints                    | Description                                         |
|-----------------|------------------------------------------------------|----------|--------------------------------|-----------------------------------------------------|
| query           | string                                               | Yes      | min: 1 char, max: 2000 chars   | Natural language question from the user (FR-011)    |
| selected_text   | string                                               | No       | max: 5000 chars                | Optional highlighted text for contextual search (FR-003) |
| module_filter   | enum: "intro", "ros2", "simulation", "isaac", "vla" | No       | Must be one of 5 valid modules | Filters retrieval to specific book module (FR-010)  |
| top_k           | integer                                              | No       | min: 1, max: 100, default: 5   | Number of retrieval results to return               |

**Validation Rules**:
- `query` must not be empty or whitespace-only (trimmed before validation)
- `selected_text` if provided, must not exceed 5000 characters
- `module_filter` if provided, must match one of the 5 book modules
- `top_k` defaults to 5, clamped to range [1, 100]

**Relationships**:
- Input to `POST /chat` endpoint
- Passed to `agent.py::ask()` function after validation

**State Transitions**: N/A (immutable request object)

---

### 2. ChatResponse

**Purpose**: Represents the structured answer returned by the `/chat` endpoint after agent processing.

**Fields**:

| Field               | Type                    | Required | Description                                              |
|---------------------|-------------------------|----------|----------------------------------------------------------|
| answer              | string                  | Yes      | AI-generated answer to the user's query                  |
| citations           | array of Citation       | Yes      | List of book sources cited in the answer (FR-004)        |
| confidence          | enum: "high", "low", "none" | Yes      | Confidence level based on retrieval relevance            |
| retrieval_time_ms   | float                   | Yes      | Time spent retrieving context from Qdrant (FR-013)       |
| generation_time_ms  | float                   | Yes      | Time spent generating answer via OpenAI agent (FR-013)   |
| total_time_ms       | float                   | Yes      | Total request processing time (FR-013)                   |
| error               | ErrorDetail (optional)  | No       | Present only if error occurred (FR-006)                  |

**Validation Rules**:
- All timing fields must be non-negative
- `citations` array may be empty if no relevant content found
- `confidence` is "none" when `citations` is empty
- `error` field only populated on partial failures (e.g., low-confidence results)

**Relationships**:
- Output from `POST /chat` endpoint (HTTP 200)
- Populated from `agent.py::AgentResponse` dataclass
- Consumed by frontend `ChatWidget` component

**State Transitions**: N/A (immutable response object)

---

### 3. Citation

**Purpose**: Represents a single source reference from the book content.

**Fields**:

| Field       | Type   | Required | Description                                      |
|-------------|--------|----------|--------------------------------------------------|
| page_title  | string | Yes      | Title of the book page (e.g., "Introduction to ROS 2") |
| page_url    | string | Yes      | Relative URL to the page (e.g., "/docs/ros2/intro") |
| module_name | string | Yes      | Book module identifier (e.g., "ros2")            |

**Validation Rules**:
- All fields are non-empty strings
- `page_url` should be a valid relative URL path
- `module_name` must match one of the 5 book modules

**Relationships**:
- Embedded in `ChatResponse.citations` array
- Derived from `agent.py::Citation` dataclass
- Used by frontend to render clickable source links

**State Transitions**: N/A (immutable)

---

### 4. ErrorDetail

**Purpose**: Represents error information in API responses.

**Fields**:

| Field      | Type   | Required | Description                                              |
|------------|--------|----------|----------------------------------------------------------|
| code       | string | Yes      | Machine-readable error code (e.g., "VALIDATION_ERROR")   |
| message    | string | Yes      | User-friendly error message (FR-006, SC-003)             |
| suggestion | string | No       | Optional actionable suggestion for the user              |

**Validation Rules**:
- `code` follows SCREAMING_SNAKE_CASE format
- `message` must not contain stack traces or internal details
- `suggestion` provides next steps (e.g., "Try rephrasing your question")

**Relationships**:
- Embedded in `ChatResponse.error` field (partial failures)
- Used standalone in HTTP 4xx/5xx error responses
- Generated by custom exception handlers

**State Transitions**: N/A (immutable)

---

### 5. HealthStatus

**Purpose**: Represents system health for monitoring endpoints.

**Fields**:

| Field    | Type                             | Required | Description                                       |
|----------|----------------------------------|----------|---------------------------------------------------|
| status   | enum: "healthy", "degraded", "unhealthy" | Yes      | Overall system status                             |
| checks   | object                           | Yes      | Component-specific health check results           |
| version  | string                           | Yes      | API version (e.g., "1.0.0")                       |

**Sub-fields in `checks`**:

| Field       | Type    | Description                                |
|-------------|---------|--------------------------------------------|
| agent       | boolean | True if agent.py module loads successfully |
| retrieval   | boolean | True if Qdrant connection is healthy       |
| env         | boolean | True if required env vars are set          |

**Validation Rules**:
- `status` is "healthy" only if all `checks` are `true`
- `status` is "degraded" if some checks fail but critical ones pass
- `status` is "unhealthy" if critical checks fail (agent, env)

**Relationships**:
- Returned by `GET /health` and `GET /ready` endpoints (FR-014)
- Must respond within 500ms (SC-008)

**State Transitions**:
```
healthy → degraded (Qdrant connection lost)
healthy → unhealthy (OpenAI API key invalid)
degraded → healthy (Qdrant reconnects)
unhealthy → healthy (Environment fixed and restarted)
```

---

## Data Flow

```
1. User submits query via ChatWidget
   ↓
2. ChatWidget sends ChatRequest to POST /chat
   ↓
3. FastAPI validates ChatRequest (Pydantic)
   ↓
4. Route handler calls agent.ask(query, selected_text, module_filter, top_k)
   ↓
5. Agent returns AgentResponse (from agent.py)
   ↓
6. Route handler maps AgentResponse → ChatResponse
   ↓
7. FastAPI serializes ChatResponse to JSON (FR-004, SC-007)
   ↓
8. ChatWidget receives ChatResponse and renders answer + citations
```

---

## Mapping to Existing Code

**From `agent.py::AgentResponse` to `ChatResponse`**:

| agent.py Field      | ChatResponse Field  | Transformation                          |
|---------------------|---------------------|-----------------------------------------|
| answer              | answer              | Direct mapping                          |
| citations           | citations           | Map each Citation to Citation model     |
| confidence          | confidence          | Direct mapping                          |
| retrieval_time_ms   | retrieval_time_ms   | Direct mapping                          |
| generation_time_ms  | generation_time_ms  | Direct mapping                          |
| total_time_ms       | total_time_ms       | Direct mapping                          |
| error               | error               | Map to ErrorDetail if present           |

**From `agent.py::Citation` to `Citation`**:

| agent.py Field | Citation Field | Transformation  |
|----------------|----------------|-----------------|
| page_title     | page_title     | Direct mapping  |
| page_url       | page_url       | Direct mapping  |
| module_name    | module_name    | Direct mapping  |

---

## Error Cases

### Validation Errors (HTTP 400)

**Trigger**: ChatRequest fails Pydantic validation
**Response**:
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Query is required and cannot be empty",
    "suggestion": "Please enter a question about the book content"
  }
}
```

---

### Agent Errors (HTTP 500)

**Trigger**: agent.ask() raises AgentError (OpenAI API failure)
**Response**:
```json
{
  "error": {
    "code": "AGENT_ERROR",
    "message": "Unable to generate a response. Please try again.",
    "suggestion": "If the problem persists, contact support"
  }
}
```

---

### Retrieval Errors (HTTP 503)

**Trigger**: agent.ask() raises RetrievalError (Qdrant unavailable)
**Response**:
```json
{
  "error": {
    "code": "RETRIEVAL_ERROR",
    "message": "Search service is temporarily unavailable",
    "suggestion": "Please try again in a few moments"
  }
}
```

---

### No Results (HTTP 200 with low confidence)

**Trigger**: agent.ask() returns confidence="none"
**Response**:
```json
{
  "answer": "I couldn't find relevant information in the book to answer your question.",
  "citations": [],
  "confidence": "none",
  "retrieval_time_ms": 145.2,
  "generation_time_ms": 320.5,
  "total_time_ms": 465.7,
  "error": null
}
```

---

## Notes

- All models defined as Pydantic v2 `BaseModel` subclasses
- OpenAPI schema auto-generated by FastAPI for frontend consumption
- No database models needed (stateless API per FR-008)
- All timestamps in milliseconds for consistency with agent.py
- Error responses follow RFC 7807 Problem Details structure (adapted)
