# Data Model: RAG Retrieval Pipeline

**Feature**: 001-rag-retrieval-pipeline
**Date**: 2025-12-27

## Entities

### SearchQuery

Represents a user's retrieval request.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| text | string | Natural language query | Non-empty, max 1000 chars |
| top_k | int | Number of results to return | 1-100, default 5 |
| module_filter | string | Filter by module name | Optional, one of: intro, ros2, simulation, isaac, vla |
| url_filter | string | Filter by page URL | Optional, valid URL pattern |

**State Transitions**: None (stateless query)

**Relationships**:
- One SearchQuery → One RetrievalResponse

---

### SearchResult

A single retrieved chunk with its metadata and relevance score.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| text | string | Chunk content | Non-empty |
| score | float | Cosine similarity score | 0.0-1.0 |
| page_title | string | Source page title | Non-empty |
| page_url | string | Source page URL | Valid URL |
| module_name | string | Module identifier | One of: intro, ros2, simulation, isaac, vla |
| chunk_index | int | Position within source page | >= 0 |

**State Transitions**: None (read-only from Qdrant)

**Relationships**:
- Many SearchResults → One RetrievalResponse
- One SearchResult ↔ One Qdrant Point (via payload)

---

### RetrievalResponse

Complete response for a search query.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| query | string | Original query text | Non-empty |
| results | list[SearchResult] | Ranked search results | 0 to top_k items |
| total_found | int | Number of results returned | >= 0 |
| query_time_ms | float | Query execution time | >= 0 |
| filters_applied | dict | Active filters | Optional |

**State Transitions**: None (response object)

**Relationships**:
- One RetrievalResponse → One SearchQuery (source)
- One RetrievalResponse → Many SearchResults (contains)

---

### Filter

Optional constraints for narrowing search scope.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| module_name | string | Module to filter by | Optional, valid module |
| page_url | string | Specific page URL | Optional, valid URL |

**Filter Logic**: AND combination (all specified filters must match)

---

### ValidationResult

Result of a single test query in validation mode.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| query | string | Test query text | Non-empty |
| expected_module | string | Expected top result module | Valid module |
| actual_module | string | Actual top result module | Valid module or null |
| top_score | float | Similarity score of top result | 0.0-1.0 or null |
| passed | bool | Whether test passed | True if expected == actual |

**State Transitions**: None (test result)

---

### ValidationReport

Summary of validation test suite execution.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| total_tests | int | Number of test queries | > 0 |
| passed | int | Number of passing tests | >= 0 |
| failed | int | Number of failing tests | >= 0 |
| accuracy | float | Pass rate percentage | 0.0-100.0 |
| results | list[ValidationResult] | Individual test results | Non-empty |
| execution_time_ms | float | Total test time | >= 0 |

---

## Qdrant Point Structure (from Spec 1)

Reference: This is the data stored in Qdrant by the ingestion pipeline.

```json
{
  "id": "UUID string (deterministic from URL + chunk_index)",
  "vector": [1024 floats],
  "payload": {
    "page_title": "string",
    "page_url": "string",
    "module_name": "string (intro|ros2|simulation|isaac|vla)",
    "chunk_index": "int",
    "text": "string"
  }
}
```

## Data Flow

```
[User Query] → [SearchQuery] → [Cohere Embed] → [Query Vector]
                                                      ↓
[Qdrant Collection] ← [Similarity Search] ← [Query Vector + Filters]
        ↓
[Qdrant Points] → [SearchResults] → [RetrievalResponse] → [CLI Output]
```

## Module Values

| Value | Description | Content Topics |
|-------|-------------|----------------|
| intro | Introduction/overview | Book overview, learning path |
| ros2 | ROS 2 Fundamentals (Module 1) | Nodes, topics, services, URDF |
| simulation | Simulation & Digital Twins (Module 2) | Gazebo, Unity, sensors |
| isaac | NVIDIA Isaac Platform (Module 3) | Isaac Sim, Isaac ROS, Nav2 |
| vla | Vision-Language-Action (Module 4) | VLA models, voice control, capstone |
