# Research: RAG Agent Construction & Retrieval Integration

**Feature**: 006-rag-agent-integration
**Date**: 2025-12-28
**Status**: Complete

## Research Summary

All technical unknowns have been resolved through documentation review and codebase analysis.

---

## R1: OpenAI Agents SDK Integration Pattern

**Question**: How to integrate OpenAI Agents SDK with the existing retrieval pipeline?

**Decision**: Use `@function_tool` decorator to wrap the retrieval pipeline as a tool, or pass retrieved context directly in the agent instructions/input.

**Rationale**:
- The OpenAI Agents SDK supports two patterns for providing context:
  1. **Tool-based**: Define a `@function_tool` that calls `retrieve()` - agent decides when to search
  2. **Direct context**: Retrieve first, then inject results into the agent input/instructions
- For RAG with grounded responses, **direct context injection** is preferred because:
  - We always want to retrieve before answering (no optional retrieval)
  - Reduces LLM calls (no tool-call round-trip)
  - Easier to enforce citation requirements in the prompt
  - More deterministic behavior

**Alternatives Considered**:
- Tool-based retrieval: Rejected because it adds latency and non-determinism
- Custom agent framework: Out of scope per spec constraints

**Source**: [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)

---

## R2: Temperature and Model Configuration

**Question**: How to configure low temperature for deterministic responses?

**Decision**: Use `ModelSettings(temperature=0.1)` in the Agent constructor.

**Rationale**:
- OpenAI Agents SDK provides `ModelSettings` class for model configuration
- Temperature 0.1 provides near-deterministic responses while allowing minimal variation
- Temperature 0.0 is also valid for fully deterministic responses

**Implementation Pattern**:
```python
from agents import Agent, ModelSettings

agent = Agent(
    name="RAG Assistant",
    instructions="...",
    model="gpt-4o-mini",  # or gpt-4o for higher quality
    model_settings=ModelSettings(temperature=0.1),
)
```

**Source**: [Model Settings Reference](https://openai.github.io/openai-agents-python/ref/model_settings/)

---

## R3: Existing Retrieval Pipeline Interface

**Question**: What is the interface of the Spec 2 retrieval pipeline?

**Decision**: Import and call `retrieve()` function from `backend/retrieve.py`.

**Rationale**: The existing implementation provides:
- `retrieve(query, config, top_k, module_filter, url_filter)` → `RetrievalResponse`
- `RetrievalResponse` contains: `query`, `results` (list of `SearchResult`), `total_found`, `query_time_ms`
- `SearchResult` contains: `text`, `score`, `page_title`, `page_url`, `module_name`, `chunk_index`
- Configuration loaded via `load_config()` and validated via `validate_config()`

**Integration Approach**:
```python
from retrieve import retrieve, load_config, validate_config, Config, RetrievalResponse

config = load_config()
validate_config(config)
response = retrieve(query="user question", config=config, top_k=5)
# Use response.results to build context for agent
```

**Source**: `backend/retrieve.py:260-323`

---

## R4: Citation Format

**Question**: How should citations be formatted in agent responses?

**Decision**: Use inline citations with page title, formatted as `[Source: Page Title]` after relevant statements.

**Rationale**:
- Page title is always available in `SearchResult.page_title`
- URL can be optionally included: `[Source: Page Title](url)`
- Consistent with academic citation patterns
- Easy to parse programmatically if needed later

**Format Options**:
1. `[Source: Introduction to ROS 2]` - Title only (default)
2. `[Introduction to ROS 2](https://...)` - Markdown link
3. `Sources: 1. Introduction to ROS 2, 2. Gazebo Setup` - Footer style

**Selected**: Option 1 (inline with title) for simplicity and readability.

---

## R5: Grounding Strategy

**Question**: How to ensure responses are strictly grounded in retrieved context?

**Decision**: Use explicit system instructions that:
1. State the agent can ONLY use provided context
2. Require citations for all factual claims
3. Instruct to decline when information is not in context
4. Include retrieved chunks directly in the prompt

**Prompt Template Pattern**:
```
You are an expert assistant for the Physical AI & Robotics book.

RULES:
1. ONLY answer using the CONTEXT provided below
2. If the answer is not in the CONTEXT, say "I couldn't find that information in the book"
3. ALWAYS cite your sources using [Source: Page Title] format
4. Be concise and accurate

CONTEXT:
{retrieved_chunks_formatted}

USER QUESTION: {query}
```

**Rationale**: Explicit instruction-based grounding is the industry standard for RAG systems without fine-tuning.

---

## R6: Error Handling Strategy

**Question**: How to handle retrieval and API failures gracefully?

**Decision**: Implement try-except blocks with specific error types and user-friendly messages.

**Error Taxonomy**:
| Error Type | Handling | User Message |
|------------|----------|--------------|
| Empty query | Validate before retrieval | "Query cannot be empty" |
| Retrieval failure | Catch exception, return error | "Search failed: {details}" |
| No results | Check `total_found == 0` | "I couldn't find relevant information..." |
| Low relevance | Check `score < threshold` | "The information may not be directly covered..." |
| OpenAI API error | Catch with retry | "Unable to generate response. Please try again." |

**Rationale**: Follows existing patterns in `backend/retrieve.py` for consistency.

---

## R7: Runner Pattern (Sync vs Async)

**Question**: Should the agent use sync or async execution?

**Decision**: Support both via `Runner.run_sync()` for CLI and `Runner.run()` for async contexts.

**Rationale**:
- `Runner.run_sync()` is simpler for backend-only CLI testing (Spec 3 scope)
- Async support (`Runner.run()`) prepares for future FastAPI integration (Spec 4)
- The SDK supports both patterns equally

**Implementation**:
```python
# Sync (for CLI/testing)
result = Runner.run_sync(agent, formatted_prompt)

# Async (for future API integration)
result = await Runner.run(agent, formatted_prompt)
```

---

## R8: Relevance Score Threshold

**Question**: What similarity score threshold indicates low relevance?

**Decision**: Use 0.3 as the threshold for "low confidence" warnings.

**Rationale**:
- Cosine similarity ranges from -1 to 1 (typically 0 to 1 for positive embeddings)
- Scores below 0.3 indicate weak semantic similarity
- This is a reasonable default; can be made configurable later

**Implementation**:
```python
MIN_RELEVANCE_SCORE = 0.3

if response.results and response.results[0].score < MIN_RELEVANCE_SCORE:
    # Add low confidence indicator
```

---

## Dependencies Confirmed

| Dependency | Version | Status |
|------------|---------|--------|
| openai-agents | 0.6.4+ | Install required |
| openai | Latest | Already installed (likely) |
| Spec 2 retrieve.py | N/A | Available at `backend/retrieve.py` |
| Python | 3.9+ | Required by SDK |

## Environment Variables Required

| Variable | Purpose | Source |
|----------|---------|--------|
| OPENAI_API_KEY | OpenAI Agents SDK | New for Spec 3 |
| COHERE_API_KEY | Query embedding | Spec 2 |
| QDRANT_URL | Vector database | Spec 1 |
| QDRANT_API_KEY | Vector database auth | Spec 1 |
| COLLECTION_NAME | Qdrant collection | Spec 1 |
