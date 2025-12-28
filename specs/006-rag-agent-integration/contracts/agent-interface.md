# Agent Interface Contract

**Feature**: 006-rag-agent-integration
**Date**: 2025-12-28
**Type**: Python Function Interface (Backend Only)

## Overview

This contract defines the programmatic interface for the RAG agent. No REST API is exposed in this spec (deferred to Spec 4).

## Primary Function

### `ask(query: str, **kwargs) -> AgentResponse`

Main entry point for querying the RAG agent.

**Parameters**:

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| query | str | Yes | - | Natural language question |
| top_k | int | No | 5 | Number of chunks to retrieve |
| module_filter | str | No | None | Filter by module name |

**Returns**: `AgentResponse`

**Raises**:
| Exception | Condition |
|-----------|-----------|
| ValueError | Query is empty or whitespace |
| RetrievalError | Retrieval pipeline failed |
| AgentError | OpenAI API failed |

**Example**:
```python
from agent import ask

# Basic usage
response = ask("What is ROS 2?")
print(response.answer)
print(response.citations)

# With options
response = ask(
    "How do I set up Gazebo?",
    top_k=10,
    module_filter="simulation"
)
```

---

## Async Variant

### `async ask_async(query: str, **kwargs) -> AgentResponse`

Async version for integration with async frameworks.

**Parameters**: Same as `ask()`

**Returns**: `AgentResponse`

**Example**:
```python
from agent import ask_async
import asyncio

async def main():
    response = await ask_async("What is ROS 2?")
    print(response.answer)

asyncio.run(main())
```

---

## Data Structures

### AgentResponse

```python
@dataclass
class AgentResponse:
    answer: str                    # Generated answer with citations
    citations: list[Citation]      # List of sources used
    query: str                     # Original question
    retrieval_time_ms: float       # Retrieval duration
    generation_time_ms: float      # LLM generation duration
    total_time_ms: float           # End-to-end duration
    confidence: str                # "high", "low", or "none"
    error: Optional[str] = None    # Error message if failed
```

### Citation

```python
@dataclass
class Citation:
    page_title: str     # Source page title
    page_url: str       # Source page URL
    module_name: str    # Module containing the source
```

---

## Error Responses

### Empty Query
```python
# Input
ask("")

# Raises
ValueError: "Query cannot be empty"
```

### No Results
```python
# Input
ask("What is quantum computing?")

# Returns
AgentResponse(
    answer="I couldn't find relevant information in the book to answer your question.",
    citations=[],
    confidence="none",
    ...
)
```

### Low Relevance
```python
# Input (tangentially related query)
ask("What about machine learning?")

# Returns
AgentResponse(
    answer="Based on the available content, [answer]...",
    citations=[...],
    confidence="low",
    ...
)
```

### Retrieval Failure
```python
# When Qdrant/Cohere unavailable
# Raises
RetrievalError: "Search failed: Connection to Qdrant timed out"
```

### API Failure
```python
# When OpenAI API unavailable
# Raises
AgentError: "Unable to generate response: API rate limit exceeded. Please try again."
```

---

## CLI Interface

### Usage
```bash
python agent.py "query text" [options]
python agent.py --validate
```

### Options
| Flag | Description |
|------|-------------|
| query | Natural language question |
| --top-k N | Number of results (default: 5) |
| --module NAME | Filter by module |
| --verbose | Show detailed output |
| --json | Output as JSON |
| --validate | Run validation test suite |

### Examples
```bash
# Basic query
python agent.py "What is ROS 2?"

# With options
python agent.py "sensors in simulation" --module simulation --top-k 10

# JSON output
python agent.py "How do VLA models work?" --json

# Validation
python agent.py --validate
```

### Exit Codes
| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | No results found |
| 2 | Configuration error |
| 3 | Retrieval or API error |
| 4 | Invalid arguments |
| 5 | Validation failed (<80% accuracy) |

---

## Validation Test Suite

### Test Queries
```python
AGENT_TEST_QUERIES = [
    ("What is ROS 2?", "ros2", True),           # Should answer
    ("How do I set up Gazebo?", "simulation", True),
    ("What is Isaac Sim?", "isaac", True),
    ("How do VLA models work?", "vla", True),
    ("What topics does this book cover?", "intro", True),
    ("What is quantum computing?", None, False), # Should decline
]
```

### Validation Criteria
1. **Relevance**: Answer relates to expected module
2. **Citations**: Response includes at least one citation
3. **Grounding**: Answer declines for out-of-scope questions
4. **Format**: Response follows expected structure

### Validation Output
```
Running Agent Validation Suite...

[PASS] "What is ROS 2?" - Answer cites ros2 module
[PASS] "How do I set up Gazebo?" - Answer cites simulation module
[PASS] "What is quantum computing?" - Correctly declined

=====================================
Validation Complete!
=====================================
  Total Tests:  6
  Passed:       6
  Failed:       0
  Accuracy:     100.0%
=====================================
```
