# Research: RAG Retrieval Pipeline

**Feature**: 001-rag-retrieval-pipeline
**Date**: 2025-12-27

## Technical Decisions

### 1. Query Embedding Strategy

**Decision**: Use Cohere embed-english-v3.0 with `input_type="search_query"`

**Rationale**:
- Must match the embedding model used in Spec 1 (ingestion)
- Cohere v3 models use different input types for documents vs queries
- `search_query` optimizes embeddings for retrieval queries
- Maintains 1024 dimension compatibility with stored vectors

**Alternatives Considered**:
- Using same `search_document` type: Rejected - Cohere recommends different types for asymmetric search
- Different embedding model: Rejected - would break compatibility with indexed vectors

### 2. File Structure

**Decision**: Single file `retrieve.py` in `backend/` directory

**Rationale**:
- User explicitly requested single file structure
- Keeps retrieval logic co-located with ingestion (`main.py`)
- Reuses existing `.env` configuration from Spec 1
- Simple CLI interface for testing

**Alternatives Considered**:
- Separate `retrieval/` package: Rejected - over-engineering for current scope
- Adding to `main.py`: Rejected - separation of concerns, different entry points

### 3. Qdrant Search Method

**Decision**: Use `QdrantClient.search()` with filter conditions

**Rationale**:
- Native Qdrant method for similarity search
- Built-in support for payload filtering
- Returns scores sorted by similarity (highest first)
- Efficient for top-k retrieval

**Alternatives Considered**:
- `search_batch()`: Rejected - not needed for single queries
- `scroll()` with manual filtering: Rejected - inefficient, no ranking

### 4. Filter Implementation

**Decision**: Qdrant `models.Filter` with `FieldCondition` for AND logic

**Rationale**:
- Native Qdrant filtering at query time
- Efficient - filtering happens during search, not post-processing
- Supports exact match on `module_name` and `page_url` fields

**Alternatives Considered**:
- Post-search filtering in Python: Rejected - inefficient, may miss relevant results
- Multiple queries per filter: Rejected - unnecessary API calls

### 5. CLI Interface Design

**Decision**: Argument-based CLI with optional flags

**Rationale**:
- Simple to use: `python retrieve.py "query text"`
- Optional flags for customization: `--top-k`, `--module`, `--url`
- `--validate` flag for running test suite
- Consistent with Spec 1 CLI pattern

**Alternatives Considered**:
- Interactive REPL mode: Rejected - out of scope, can add later
- Config file for queries: Rejected - over-engineering

### 6. Validation Test Suite

**Decision**: Predefined test queries with expected module mapping

**Rationale**:
- Deterministic test cases for each module
- Easy to verify retrieval accuracy
- Summary report with pass/fail counts
- 80% accuracy threshold per spec requirements

**Test Queries**:
| Query | Expected Module |
|-------|-----------------|
| "What is ROS 2 and how do nodes communicate?" | ros2 |
| "How do I set up Gazebo for robot simulation?" | simulation |
| "What is NVIDIA Isaac Sim and Isaac ROS?" | isaac |
| "How do VLA models work for robot control?" | vla |
| "What topics does this book cover?" | intro |

**Alternatives Considered**:
- External test file: Rejected - adds complexity, embedded queries sufficient
- Random query generation: Rejected - non-deterministic, hard to validate

## Dependencies Research

### Qdrant Client Best Practices

- Use `with_payload=True` to get metadata with results
- Use `with_vectors=False` to reduce response size (we don't need vectors back)
- `score_threshold` can filter low-relevance results (optional)
- `limit` parameter for top-k control

### Cohere Embed API Best Practices

- Use `input_type="search_query"` for queries (different from `search_document`)
- Single text embedding (not batch) for query use case
- Handle `TooManyRequestsError` with retry logic

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Query embedding fails | Retry with backoff, clear error message |
| No results found | Return empty list with informative message |
| Qdrant connection fails | Validate connection on startup, fail fast |
| Low relevance results | Return similarity scores, let user interpret |
