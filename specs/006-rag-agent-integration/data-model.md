# Data Model: RAG Agent Construction & Retrieval Integration

**Feature**: 006-rag-agent-integration
**Date**: 2025-12-28

## Entity Overview

```
┌─────────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│     Query       │────▶│  RetrievalContext   │────▶│   AgentResponse  │
└─────────────────┘     └─────────────────────┘     └──────────────────┘
                               │                            │
                               ▼                            ▼
                        ┌─────────────┐              ┌──────────────┐
                        │SearchResult │              │   Citation   │
                        │  (Spec 2)   │              └──────────────┘
                        └─────────────┘
```

## Entities

### Query

Input entity representing a user's natural language question.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| text | str | Yes | The natural language question |
| top_k | int | No | Number of chunks to retrieve (default: 5) |
| module_filter | str | No | Filter by module name (intro, ros2, simulation, isaac, vla) |

**Validation Rules**:
- `text` must not be empty or whitespace only
- `top_k` must be between 1 and 100 (inherited from Spec 2)
- `module_filter` must be one of the valid module names if provided

### SearchResult (from Spec 2)

Re-used entity from the retrieval pipeline representing a single retrieved chunk.

| Field | Type | Description |
|-------|------|-------------|
| text | str | The chunk content |
| score | float | Cosine similarity score (0-1) |
| page_title | str | Source page title |
| page_url | str | Source page URL |
| module_name | str | Module name (intro, ros2, simulation, isaac, vla) |
| chunk_index | int | Position within the source page |

### RetrievalContext

Container for retrieval results passed to the agent.

| Field | Type | Description |
|-------|------|-------------|
| query | str | Original query text |
| chunks | list[SearchResult] | Retrieved chunks from Spec 2 |
| total_found | int | Number of results returned |
| query_time_ms | float | Retrieval execution time |
| has_relevant_results | bool | True if any chunk has score >= MIN_RELEVANCE_SCORE |

**Derived Fields**:
- `has_relevant_results`: Computed from `chunks[0].score >= 0.3` if chunks exist

### Citation

A reference to a source used in the agent's response.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| page_title | str | Yes | Title of the source page |
| page_url | str | No | URL of the source page |
| module_name | str | Yes | Module containing the source |

**Derivation**: Citations are extracted from `SearchResult` objects used to generate the answer.

### AgentResponse

Complete response from the RAG agent.

| Field | Type | Description |
|-------|------|-------------|
| answer | str | Generated answer text with inline citations |
| citations | list[Citation] | List of sources used |
| query | str | Original user question |
| retrieval_time_ms | float | Time spent on retrieval |
| generation_time_ms | float | Time spent on LLM generation |
| total_time_ms | float | End-to-end response time |
| confidence | str | "high", "low", or "none" based on relevance scores |
| error | str | None | Error message if any failure occurred |

**Confidence Levels**:
- `"high"`: Top result score >= 0.3
- `"low"`: Top result score < 0.3 but > 0
- `"none"`: No results returned

## State Transitions

```
                    ┌─────────────┐
                    │   PENDING   │
                    └──────┬──────┘
                           │ validate_query()
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
    ┌──────────┐    ┌─────────────┐   ┌──────────┐
    │  ERROR   │    │ RETRIEVING  │   │  ERROR   │
    │(empty)   │    └──────┬──────┘   │(invalid) │
    └──────────┘           │          └──────────┘
                           │ retrieve()
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
    ┌──────────┐    ┌─────────────┐   ┌──────────┐
    │  ERROR   │    │ GENERATING  │   │ NO_RESULTS│
    │(retrieval)│   └──────┬──────┘   └──────────┘
    └──────────┘           │
                           │ agent.run()
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
    ┌──────────┐    ┌─────────────┐   ┌──────────┐
    │  ERROR   │    │  COMPLETED  │   │ LOW_CONF │
    │(api)     │    └─────────────┘   └──────────┘
    └──────────┘
```

## Data Flow

### Request Flow
```
1. User provides Query
2. Query.text validated (non-empty)
3. retrieve(Query.text, top_k, module_filter) called
4. RetrievalResponse from Spec 2 converted to RetrievalContext
5. Context formatted into prompt with chunks
6. Agent.run() generates answer
7. Citations extracted from used chunks
8. AgentResponse constructed and returned
```

### Prompt Construction
```python
def build_prompt(context: RetrievalContext) -> str:
    chunks_text = "\n\n".join([
        f"[{i+1}] {chunk.page_title}\n{chunk.text}"
        for i, chunk in enumerate(context.chunks)
    ])

    return f"""
You are an expert assistant for the Physical AI & Robotics book.

RULES:
1. ONLY answer using the CONTEXT provided below
2. If the answer is not in the CONTEXT, say "I couldn't find that information in the book"
3. ALWAYS cite your sources using [Source: Page Title] format
4. Be concise and accurate

CONTEXT:
{chunks_text}

USER QUESTION: {context.query}
"""
```

## Configuration

### AgentConfig

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| model | str | "gpt-4o-mini" | OpenAI model to use |
| temperature | float | 0.1 | Response randomness (low for determinism) |
| top_k | int | 5 | Default chunks to retrieve |
| min_relevance_score | float | 0.3 | Threshold for confidence |
| openai_api_key | str | env | From OPENAI_API_KEY |

## Relationship to Spec 2

This spec **extends** the Spec 2 data model:

| Spec 2 Entity | Spec 3 Usage |
|---------------|--------------|
| Config | Reused directly for retrieval |
| SearchResult | Converted to Citation |
| RetrievalResponse | Wrapped in RetrievalContext |

No modifications to Spec 2 entities are required.
