# Implementation Plan: RAG Agent Construction & Retrieval Integration

**Branch**: `006-rag-agent-integration` | **Date**: 2025-12-28 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/006-rag-agent-integration/spec.md`

## Summary

Build an intelligent RAG agent using the OpenAI Agents SDK that accepts natural language questions, retrieves relevant content from the Spec 2 pipeline, and generates grounded answers with citations. The agent enforces strict grounding constraints, includes citations in every response, and handles multi-module queries by synthesizing information from multiple sources.

**Technical Approach**: Direct context injection pattern - retrieve chunks first, then pass as formatted context to the agent with explicit grounding instructions. Use `ModelSettings(temperature=0.1)` for deterministic responses and implement comprehensive error handling for retrieval and API failures.

## Technical Context

**Language/Version**: Python 3.9+ (required by OpenAI Agents SDK)
**Primary Dependencies**: openai-agents (0.6.4+), openai, cohere, qdrant-client, python-dotenv
**Storage**: Qdrant Cloud (vector database from Spec 1), no additional storage required
**Testing**: pytest (unit tests), validation test suite (integration tests)
**Target Platform**: Linux/Windows CLI, backend function interface
**Project Type**: Single backend project (extends existing `backend/` directory)
**Performance Goals**: <10s end-to-end latency, <2s retrieval, <8s generation
**Constraints**: Backend-only (no API/UI in this spec), strict grounding (no hallucinations), 100% citation coverage
**Scale/Scope**: ~200 lines of code in `backend/agent.py`, 5-10 test cases, single-file implementation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Content Accuracy (Principle II)
- **Requirement**: RAG chatbot MUST only answer using retrieved content, no hallucinations
- **Implementation**: Explicit system instructions enforce grounding, citations required for all claims
- **Validation**: Test suite includes out-of-scope queries to verify decline behavior

### ✅ Retrieval Transparency (Principle IV)
- **Requirement**: Citations MUST be clearly included with page/section references
- **Implementation**: Citations extracted from SearchResult metadata, formatted as `[Source: Page Title]`
- **Validation**: 100% citation coverage in all responses using retrieved content

### ✅ Test-Driven Quality (Principle VI)
- **Requirement**: Features MUST include testable acceptance criteria
- **Implementation**: Validation test suite with known Q&A pairs, edge case coverage
- **Validation**: CLI `--validate` mode runs comprehensive test suite

### ✅ Spec-Driven Reproducibility (Principle I)
- **Requirement**: All workflows MUST be reproducible from specs
- **Implementation**: Plan follows research.md, data-model.md, contracts/, quickstart.md structure
- **Validation**: Single-command setup after environment configuration

### ✅ Technology Standards
- **Requirement**: Use OpenAI Agents SDK per constitution RAG stack
- **Implementation**: OpenAI Agents SDK with direct context injection pattern
- **Validation**: Follows SDK best practices from research phase

**Result**: ✅ All constitution checks PASSED - no violations

## Project Structure

### Documentation (this feature)

```text
specs/006-rag-agent-integration/
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0 output - completed
├── data-model.md        # Phase 1 output - completed
├── quickstart.md        # Phase 1 output - completed
├── contracts/           # Phase 1 output - completed
│   └── agent-interface.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── retrieve.py          # Existing (Spec 2) - NO CHANGES
├── agent.py             # NEW - Main agent implementation
├── main.py              # Existing - MAY IMPORT agent.py for future use
└── .env                 # UPDATED - Add OPENAI_API_KEY

tests/
└── agent/               # NEW
    ├── test_agent.py           # Unit tests for agent functions
    ├── test_integration.py     # Integration tests with retrieval
    └── test_validation.py      # Validation suite tests
```

**Structure Decision**: Extends the existing single backend project structure from Spec 2. No new directories or complex layouts required. Agent implementation is self-contained in `backend/agent.py` as a single module with clear function boundaries.

## Complexity Tracking

> **No constitution violations** - This section intentionally left empty per constitution compliance.

---

## Architecture Decisions

### AD-1: Direct Context Injection vs Tool-Based Retrieval

**Decision**: Use direct context injection - retrieve chunks first, then pass formatted context to agent.

**Rationale**:
- Reduces latency (no tool-call round-trip to LLM)
- More deterministic behavior (always retrieves before answering)
- Easier to enforce citation requirements in prompt
- Simpler implementation for backend-only scope

**Alternatives Rejected**:
- Tool-based retrieval (`@function_tool`): Adds latency and non-determinism, agent might skip retrieval
- Custom RAG framework: Out of scope, violates technology standards

**Implementation**:
```python
# 1. Retrieve first
retrieval_response = retrieve(query, config, top_k)

# 2. Format context
context = format_context(retrieval_response)

# 3. Pass to agent
prompt = build_prompt(context, query)
result = Runner.run_sync(agent, prompt)
```

---

### AD-2: Citation Format

**Decision**: Inline citations formatted as `[Source: Page Title]` after relevant statements.

**Rationale**:
- Page title always available in SearchResult metadata
- Readable and concise
- Easy to parse programmatically if needed
- Consistent with academic citation patterns

**Alternatives Rejected**:
- Markdown links: Adds complexity, not all outputs support hyperlinks
- Footer-style citations: Disconnects sources from statements

**Example Output**:
```
ROS 2 is a framework for building robot applications [Source: Introduction to ROS 2].
It uses a distributed architecture with nodes [Source: ROS 2 Architecture Guide].
```

---

### AD-3: Temperature Configuration

**Decision**: Use `temperature=0.1` for near-deterministic responses.

**Rationale**:
- Spec requirement FR-005: low temperature (0.0-0.2) for deterministic responses
- 0.1 allows minimal variation while maintaining response quality
- 0.0 is too restrictive and can produce robotic outputs

**Implementation**:
```python
from agents import Agent, ModelSettings

agent = Agent(
    name="RAG Assistant",
    instructions="...",
    model="gpt-4o-mini",
    model_settings=ModelSettings(temperature=0.1),
)
```

---

### AD-4: Error Handling Strategy

**Decision**: Explicit exception types for different failure modes with user-friendly messages.

**Error Taxonomy**:
| Condition | Exception | User Message |
|-----------|-----------|--------------|
| Empty query | ValueError | "Query cannot be empty" |
| Retrieval failure | RetrievalError | "Search failed: {details}" |
| No results | None (normal response) | "I couldn't find relevant information..." |
| Low relevance | None (confidence indicator) | Answer + "may not be directly covered" |
| OpenAI API error | AgentError | "Unable to generate response. Please try again." |

**Rationale**: Follows Spec 2 patterns for consistency, provides actionable feedback.

---

### AD-5: Sync vs Async API

**Decision**: Provide both `ask()` (sync) and `ask_async()` (async) functions.

**Rationale**:
- Sync is simpler for CLI testing (Spec 3 scope)
- Async prepares for future FastAPI integration (Spec 4)
- OpenAI Agents SDK supports both patterns equally

**Implementation**:
```python
def ask(query: str, **kwargs) -> AgentResponse:
    """Sync interface for CLI/testing"""
    return Runner.run_sync(agent, build_prompt(...))

async def ask_async(query: str, **kwargs) -> AgentResponse:
    """Async interface for future API integration"""
    return await Runner.run(agent, build_prompt(...))
```

---

### AD-6: Relevance Score Threshold

**Decision**: Use `MIN_RELEVANCE_SCORE = 0.3` for low confidence indicator.

**Rationale**:
- Cosine similarity < 0.3 indicates weak semantic match
- Reasonable default, can be made configurable later
- Allows agent to answer but signals uncertainty

**Implementation**:
```python
confidence = "high"
if response.results and response.results[0].score < 0.3:
    confidence = "low"
elif not response.results:
    confidence = "none"
```

---

### AD-7: Agent Model Selection

**Decision**: Use `gpt-4o-mini` as default, allow override via environment variable.

**Rationale**:
- `gpt-4o-mini` provides good balance of quality and cost
- Faster responses than `gpt-4o` (latency target <10s)
- Can upgrade to `gpt-4o` if quality issues arise
- Configurable via `OPENAI_MODEL` env var

**Configuration**:
```python
DEFAULT_MODEL = "gpt-4o-mini"
model = os.getenv("OPENAI_MODEL", DEFAULT_MODEL)

agent = Agent(
    name="RAG Assistant",
    model=model,
    ...
)
```

---

## Key Modules

### Module: backend/agent.py

**Purpose**: Main agent implementation with retrieval integration

**Public Functions**:
- `ask(query: str, **kwargs) -> AgentResponse`: Sync query interface
- `ask_async(query: str, **kwargs) -> AgentResponse`: Async query interface
- `main()`: CLI entry point

**Internal Functions**:
- `load_agent_config() -> AgentConfig`: Load configuration from environment
- `validate_agent_config(config: AgentConfig) -> None`: Validate API keys
- `build_prompt(context: RetrievalContext, query: str) -> str`: Format prompt with context
- `format_context(response: RetrievalResponse) -> RetrievalContext`: Convert Spec 2 response
- `extract_citations(response: RetrievalResponse) -> list[Citation]`: Build citation list
- `run_agent_validation() -> ValidationReport`: Execute test suite

**Data Classes**:
- `AgentConfig`: Configuration (model, temperature, API keys)
- `AgentResponse`: Complete response with answer, citations, timing
- `Citation`: Source reference (page_title, page_url, module_name)

**Dependencies**:
- `retrieve.py`: Import `retrieve()`, `load_config()`, `validate_config()`, data classes
- `openai-agents`: Import `Agent`, `Runner`, `ModelSettings`

---

## Data Flow

```
┌─────────────────┐
│  User Question  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Validate Query  │ (non-empty)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ retrieve()      │ (Spec 2 function)
│ • Embed query   │
│ • Search Qdrant │
│ • Return chunks │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Format Context  │ (build_prompt)
│ • Number chunks │
│ • Add titles    │
│ • Inject rules  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Agent.run()     │ (OpenAI Agents SDK)
│ • Call LLM      │
│ • Generate ans. │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│Extract Citations│ (from used chunks)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ AgentResponse   │ (return to user)
└─────────────────┘
```

---

## Testing Strategy

### Unit Tests (tests/agent/test_agent.py)

- `test_validate_query_empty()`: Raises ValueError for empty query
- `test_validate_query_whitespace()`: Raises ValueError for whitespace
- `test_format_context()`: Correctly formats RetrievalResponse
- `test_extract_citations()`: Extracts citations from SearchResults
- `test_build_prompt()`: Includes context and rules in prompt
- `test_confidence_high()`: Sets confidence="high" for score >= 0.3
- `test_confidence_low()`: Sets confidence="low" for score < 0.3
- `test_confidence_none()`: Sets confidence="none" for no results

### Integration Tests (tests/agent/test_integration.py)

- `test_ask_ros2_question()`: End-to-end for ROS 2 query
- `test_ask_with_citations()`: Verifies citations in response
- `test_ask_out_of_scope()`: Declines quantum computing query
- `test_ask_multi_module()`: Handles cross-module queries
- `test_retrieval_failure()`: Handles Qdrant connection error
- `test_api_failure()`: Handles OpenAI API error

### Validation Tests (tests/agent/test_validation.py)

- `test_run_validation_suite()`: Executes all validation queries
- `test_validation_accuracy_threshold()`: Passes if accuracy >= 90%
- `test_citation_coverage()`: All answers have citations
- `test_grounding_constraint()`: No hallucinations detected

---

## Deployment Considerations

### Environment Variables

Required additions to `.env`:
```bash
# New for Spec 3
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o-mini  # Optional, defaults to gpt-4o-mini
MIN_RELEVANCE_SCORE=0.3   # Optional, defaults to 0.3

# Existing from Spec 1 & 2 (no changes)
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
COLLECTION_NAME=physical-ai-book
```

### Package Dependencies

Add to `backend/requirements.txt`:
```
openai-agents>=0.6.4
```

### CLI Installation

No changes to installation process:
```bash
cd backend
pip install -r requirements.txt
python agent.py --help
```

---

## Success Metrics

| Metric | Target | How Measured |
|--------|--------|--------------|
| Response relevance | 90% | Validation test pass rate |
| Citation coverage | 100% | All responses with content have ≥1 citation |
| Out-of-scope handling | 95% | Agent declines quantum computing, etc. |
| End-to-end latency | <10s | Timer in AgentResponse |
| Grounding constraint | 100% | Zero hallucinations in validation |
| Multi-module queries | Works | Cross-module test cases pass |
| Deterministic responses | Yes | Same query → same answer (temp=0.1) |

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| OpenAI API rate limits | High | Implement exponential backoff, clear error messages |
| Hallucinations despite grounding | Critical | Strong system instructions, validation suite, manual spot-checks |
| Citation extraction failures | Medium | Fallback to "Unknown Source" if metadata missing |
| Latency > 10s target | Medium | Profile retrieval vs generation, optimize retrieval top_k |
| Temperature 0.1 too low (robotic) | Low | Test with 0.2, make configurable via env var |

---

## Phase 0 Output

✅ **research.md** - Completed
- All NEEDS CLARIFICATION items resolved
- Integration patterns decided
- Temperature configuration confirmed
- Error handling strategy defined

## Phase 1 Output

✅ **data-model.md** - Completed
- Entities: Query, SearchResult, RetrievalContext, Citation, AgentResponse
- State transitions defined
- Data flow documented

✅ **contracts/agent-interface.md** - Completed
- `ask()` and `ask_async()` interfaces defined
- CLI interface specified
- Error responses documented

✅ **quickstart.md** - Completed
- Installation steps
- Basic usage examples
- Validation instructions

---

## Next Steps

After `/sp.plan` completion:
1. Run `/sp.tasks` to generate tasks.md with implementation tasks
2. Execute `/sp.implement` to build agent.py following TDD
3. Run validation suite to verify all success criteria
4. Document results in ADR if architectural decisions warrant it

---

## References

- [Spec 3: Feature Specification](spec.md)
- [Research Phase](research.md)
- [Data Model](data-model.md)
- [Agent Interface Contract](contracts/agent-interface.md)
- [Quickstart Guide](quickstart.md)
- [OpenAI Agents SDK Docs](https://openai.github.io/openai-agents-python/)
- [Spec 2: Retrieval Pipeline](../001-rag-retrieval-pipeline/spec.md)
