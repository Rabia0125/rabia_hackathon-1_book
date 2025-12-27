# Quickstart: RAG Retrieval Pipeline

**Feature**: 001-rag-retrieval-pipeline
**Date**: 2025-12-27

## Prerequisites

1. **Spec 1 Complete**: The ingestion pipeline (`001-rag-data-pipeline`) must have been executed with vectors stored in Qdrant Cloud.

2. **Environment Setup**: The `.env` file in `backend/` must contain:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   COLLECTION_NAME=physical-ai-book
   ```

3. **Python Environment**: UV package manager installed with Python 3.11+

## Installation

No additional installation required - uses same dependencies as Spec 1.

```bash
cd backend
# Dependencies already installed from Spec 1
```

## Quick Start

### Basic Query

```bash
uv run python retrieve.py "What is ROS 2?"
```

Expected output:
```
Query: "What is ROS 2?"
Found 5 results in 234ms

[1] Score: 0.892 | Module: ros2
    Title: ROS 2 Fundamentals
    URL: local://docs/docs/module-1-ros2/01-ros2-fundamentals
...
```

### Query with Filters

```bash
# Filter by module
uv run python retrieve.py "sensors" --module simulation

# Get more results
uv run python retrieve.py "robot navigation" --top-k 10

# Combine filters
uv run python retrieve.py "URDF" --module ros2 --top-k 3
```

### Output Formats

```bash
# Verbose output (includes chunk text)
uv run python retrieve.py "Isaac Sim" --verbose

# JSON output (for programmatic use)
uv run python retrieve.py "digital twin" --json
```

### Validation

Run the validation test suite to verify retrieval accuracy:

```bash
uv run python retrieve.py --validate
```

Expected output:
```
Running Retrieval Validation Suite...

[PASS] "What is ROS 2 and how do nodes communicate?"
       Expected: ros2 | Actual: ros2 | Score: 0.892
...

==================================================
Validation Complete!
==================================================
  Total Tests:  5
  Passed:       5
  Failed:       0
  Accuracy:     100.0%
==================================================
```

## Command Reference

| Command | Description |
|---------|-------------|
| `retrieve.py "query"` | Basic query |
| `retrieve.py "query" --top-k N` | Return N results (1-100) |
| `retrieve.py "query" --module NAME` | Filter by module |
| `retrieve.py "query" --url URL` | Filter by page URL |
| `retrieve.py "query" --verbose` | Show chunk text |
| `retrieve.py "query" --json` | JSON output |
| `retrieve.py --validate` | Run test suite |

## Module Names

| Module | Description |
|--------|-------------|
| `intro` | Introduction/overview |
| `ros2` | ROS 2 Fundamentals (Module 1) |
| `simulation` | Simulation & Digital Twins (Module 2) |
| `isaac` | NVIDIA Isaac Platform (Module 3) |
| `vla` | Vision-Language-Action (Module 4) |

## Troubleshooting

### "Configuration error: COHERE_API_KEY is required"
- Ensure `.env` file exists in `backend/` directory
- Check that all required environment variables are set

### "Cannot connect to Qdrant"
- Verify `QDRANT_URL` is correct
- Check internet connectivity
- Verify Qdrant Cloud cluster is running

### "No results found"
- Try a broader query
- Remove filters to search all content
- Run `--validate` to verify retrieval is working

### Low similarity scores
- Scores below 0.5 may indicate irrelevant results
- Try rephrasing the query
- Use module filter to narrow scope
