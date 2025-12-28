# Quickstart: RAG Agent Construction & Retrieval Integration

**Feature**: 006-rag-agent-integration
**Date**: 2025-12-28

## Prerequisites

- Python 3.9 or newer
- Completed Spec 2 (001-rag-retrieval-pipeline) with indexed vectors in Qdrant
- Environment variables configured in `.env` file

## Installation

### 1. Install Dependencies

```bash
cd backend
pip install openai-agents
```

**Note**: Existing dependencies (cohere, qdrant-client, python-dotenv) are already installed from Spec 2.

### 2. Configure Environment

Add OpenAI API key to `.env`:

```bash
# .env file
OPENAI_API_KEY=sk-...

# Existing from Spec 1 & 2
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
COLLECTION_NAME=physical-ai-book
```

### 3. Verify Setup

```bash
python agent.py --help
```

Expected output:
```
usage: agent.py [-h] [--top-k N] [--module NAME] [--verbose] [--json] [--validate] [query]

RAG Agent for Physical AI & Robotics Book
```

---

## Basic Usage

### Ask a Question

```bash
python agent.py "What is ROS 2?"
```

Expected output:
```
Query: "What is ROS 2?"
Confidence: high

Answer:
ROS 2 (Robot Operating System 2) is a framework for building robot applications... [Source: Introduction to ROS 2]

Citations:
  1. Introduction to ROS 2 (ros2 module)
     https://book-url.com/ros2/intro

Response Time: 1.2s (retrieval: 0.5s, generation: 0.7s)
```

### Filter by Module

```bash
python agent.py "sensors in robotics" --module simulation
```

### Increase Context

```bash
python agent.py "How do VLA models work?" --top-k 10
```

### JSON Output

```bash
python agent.py "What is Isaac Sim?" --json
```

Expected output:
```json
{
  "answer": "Isaac Sim is... [Source: Isaac Overview]",
  "citations": [
    {
      "page_title": "Isaac Overview",
      "page_url": "https://...",
      "module_name": "isaac"
    }
  ],
  "query": "What is Isaac Sim?",
  "retrieval_time_ms": 450.2,
  "generation_time_ms": 680.5,
  "total_time_ms": 1130.7,
  "confidence": "high",
  "error": null
}
```

---

## Programmatic Usage

### Python Script

```python
from agent import ask

# Basic query
response = ask("What is ROS 2?")
print(response.answer)

# With options
response = ask(
    "How do I set up Gazebo?",
    top_k=10,
    module_filter="simulation"
)

# Check confidence
if response.confidence == "low":
    print("Note: Low confidence answer")

# Access citations
for citation in response.citations:
    print(f"- {citation.page_title} ({citation.module_name})")
```

### Async Usage (Future-Ready)

```python
import asyncio
from agent import ask_async

async def main():
    response = await ask_async("What is ROS 2?")
    print(response.answer)

asyncio.run(main())
```

---

## Validation & Testing

### Run Validation Suite

```bash
python agent.py --validate
```

Expected output:
```
Running Agent Validation Suite...

[PASS] "What is ROS 2?" - Answer cites ros2 module
[PASS] "How do I set up Gazebo?" - Answer cites simulation module
[PASS] "What is Isaac Sim?" - Answer cites isaac module
[PASS] "How do VLA models work?" - Answer cites vla module
[PASS] "What topics does this book cover?" - Answer cites intro module
[PASS] "What is quantum computing?" - Correctly declined

=====================================
Validation Complete!
=====================================
  Total Tests:  6
  Passed:       6
  Failed:       0
  Accuracy:     100.0%
  Duration:     8.5s
=====================================
```

### Custom Test Query

```python
from agent import ask

# Test grounding constraint
response = ask("What is quantum computing?")
assert "couldn't find" in response.answer.lower()
assert response.confidence == "none"

# Test citation requirement
response = ask("What is ROS 2?")
assert len(response.citations) > 0
```

---

## Expected Behavior

### ✅ Successful Query
- Returns answer with inline citations
- Citations list includes all sources used
- Confidence indicator reflects relevance
- Response time < 10 seconds

### ⚠️ Low Confidence
- Answer includes "may not be directly covered" disclaimer
- Citations provided but relevance scores < 0.3
- Confidence set to "low"

### ❌ Out of Scope
- Response: "I couldn't find relevant information..."
- Empty citations list
- Confidence set to "none"

### ❌ Error Conditions
- Empty query → ValueError
- Retrieval failure → RetrievalError
- API failure → AgentError

---

## Troubleshooting

### "OPENAI_API_KEY is required but not set"
**Solution**: Add `OPENAI_API_KEY=sk-...` to `.env` file

### "COHERE_API_KEY is required but not set"
**Solution**: Spec 2 environment not configured. See Spec 2 quickstart.

### "Connection to Qdrant timed out"
**Solution**: Verify Qdrant Cloud cluster is running and `QDRANT_URL` is correct

### "Rate limit exceeded"
**Solution**: Wait and retry. Implement backoff in production usage.

### Agent returns ungrounded answers
**Solution**: Verify prompt instructions in `agent.py`. Check citations are required.

---

## Performance Benchmarks

| Metric | Target | Typical |
|--------|--------|---------|
| End-to-end latency | < 10s | 1-3s |
| Retrieval time | < 2s | 0.5-1s |
| Generation time | < 8s | 0.7-2s |
| Accuracy (validation) | > 90% | 100% |
| Citation coverage | 100% | 100% |

---

## Next Steps

After validating the agent:
1. **Spec 4**: Expose via FastAPI endpoints
2. **Spec 5**: Frontend UI integration
3. **Future**: Multi-turn conversation support
4. **Future**: Response streaming

---

## References

- [Spec 2: Retrieval Pipeline](../001-rag-retrieval-pipeline/spec.md)
- [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)
- [Project Constitution](../../../.specify/memory/constitution.md)
