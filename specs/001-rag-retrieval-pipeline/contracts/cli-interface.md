# CLI Interface Contract: RAG Retrieval Pipeline

**Feature**: 001-rag-retrieval-pipeline
**Date**: 2025-12-27

## Overview

This retrieval pipeline is a CLI tool for querying the indexed book content. It's used for testing and validation.

## Entry Point

```bash
uv run python retrieve.py "query text" [options]
```

## Command Line Arguments

### Required

| Argument | Type | Description |
|----------|------|-------------|
| `query` | string | Natural language query (positional) |

### Optional Flags

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--top-k` | int | 5 | Number of results to return (1-100) |
| `--module` | string | none | Filter by module (intro, ros2, simulation, isaac, vla) |
| `--url` | string | none | Filter by page URL (exact match) |
| `--validate` | flag | false | Run validation test suite instead of query |
| `--verbose` | flag | false | Show detailed output including chunk text |
| `--json` | flag | false | Output results as JSON |

## Environment Variables (Required)

Same as Spec 1 - loaded from `.env`:

| Variable | Description | Example |
|----------|-------------|---------|
| `COHERE_API_KEY` | Cohere API authentication | `co_xxxxxxxxxxxx` |
| `QDRANT_URL` | Qdrant Cloud cluster URL | `https://xxx.cloud.qdrant.io:6333` |
| `QDRANT_API_KEY` | Qdrant Cloud authentication | `qdrant_xxxxxxxxxxxx` |
| `COLLECTION_NAME` | Qdrant collection name | `physical-ai-book` |

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success - query executed, results returned |
| 1 | No results found (not an error) |
| 2 | Configuration error - missing env vars |
| 3 | Connection error - cannot reach Qdrant or Cohere |
| 4 | Validation error - invalid arguments |
| 5 | Validation failed - test suite did not pass threshold |

## Output Formats

### Standard Output (Default)

```
Query: "What is ROS 2?"
Found 5 results in 234ms

[1] Score: 0.892 | Module: ros2
    Title: ROS 2 Fundamentals
    URL: local://docs/docs/module-1-ros2/01-ros2-fundamentals

[2] Score: 0.845 | Module: ros2
    Title: Python ROS Control
    URL: local://docs/docs/module-1-ros2/02-python-ros-control

[3] Score: 0.823 | Module: ros2
    Title: Humanoid URDF
    URL: local://docs/docs/module-1-ros2/03-humanoid-urdf

[4] Score: 0.756 | Module: simulation
    Title: Gazebo Simulation
    URL: local://docs/docs/module-2-simulation/01-gazebo-simulation

[5] Score: 0.734 | Module: intro
    Title: Introduction
    URL: local://docs/docs/intro
```

### Verbose Output (--verbose)

Includes chunk text preview:

```
[1] Score: 0.892 | Module: ros2
    Title: ROS 2 Fundamentals
    URL: local://docs/docs/module-1-ros2/01-ros2-fundamentals
    Text: "ROS 2 (Robot Operating System 2) is an open-source robotics
          middleware that provides tools, libraries, and conventions for
          building robot applications..."
```

### JSON Output (--json)

```json
{
  "query": "What is ROS 2?",
  "total_found": 5,
  "query_time_ms": 234,
  "filters": {},
  "results": [
    {
      "score": 0.892,
      "module_name": "ros2",
      "page_title": "ROS 2 Fundamentals",
      "page_url": "local://docs/docs/module-1-ros2/01-ros2-fundamentals",
      "chunk_index": 0,
      "text": "ROS 2 (Robot Operating System 2)..."
    }
  ]
}
```

### Validation Output (--validate)

```
Running Retrieval Validation Suite...

[PASS] "What is ROS 2 and how do nodes communicate?"
       Expected: ros2 | Actual: ros2 | Score: 0.892

[PASS] "How do I set up Gazebo for robot simulation?"
       Expected: simulation | Actual: simulation | Score: 0.867

[PASS] "What is NVIDIA Isaac Sim and Isaac ROS?"
       Expected: isaac | Actual: isaac | Score: 0.845

[PASS] "How do VLA models work for robot control?"
       Expected: vla | Actual: vla | Score: 0.823

[PASS] "What topics does this book cover?"
       Expected: intro | Actual: intro | Score: 0.789

==================================================
Validation Complete!
==================================================
  Total Tests:  5
  Passed:       5
  Failed:       0
  Accuracy:     100.0%
  Duration:     1.23s
==================================================
```

## Error Output (stderr)

```
[ERROR] Configuration error: COHERE_API_KEY is required but not set
[ERROR] Connection error: Cannot connect to Qdrant at https://...
[ERROR] Invalid argument: --top-k must be between 1 and 100
[ERROR] Query cannot be empty
```

## Usage Examples

```bash
# Basic query
uv run python retrieve.py "What is ROS 2?"

# Query with top-k
uv run python retrieve.py "robot navigation" --top-k 10

# Query filtered by module
uv run python retrieve.py "sensors" --module simulation

# Query with multiple filters
uv run python retrieve.py "URDF" --module ros2 --top-k 3

# Verbose output with chunk text
uv run python retrieve.py "Isaac Sim" --verbose

# JSON output for programmatic use
uv run python retrieve.py "digital twin" --json

# Run validation test suite
uv run python retrieve.py --validate

# Verbose validation
uv run python retrieve.py --validate --verbose
```
