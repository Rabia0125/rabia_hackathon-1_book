# CLI Interface Contract: RAG Data Pipeline

**Feature**: 001-rag-data-pipeline
**Date**: 2025-12-26

## Overview

This pipeline is a CLI tool (not an API server). It's run on-demand to index book content.

## Entry Point

```bash
uv run python main.py [options]
```

## Command Line Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--verbose` | flag | false | Enable detailed logging |
| `--dry-run` | flag | false | Scrape and chunk without embedding/storing |
| `--module` | string | all | Process only specific module (intro, ros2, simulation, isaac, vla) |

## Environment Variables (Required)

| Variable | Description | Example |
|----------|-------------|---------|
| `COHERE_API_KEY` | Cohere API authentication | `co_xxxxxxxxxxxx` |
| `QDRANT_URL` | Qdrant Cloud cluster URL | `https://xxx.cloud.qdrant.io:6333` |
| `QDRANT_API_KEY` | Qdrant Cloud authentication | `qdrant_xxxxxxxxxxxx` |

## Environment Variables (Optional)

| Variable | Default | Description |
|----------|---------|-------------|
| `TARGET_URL` | `https://rabia-hackathon-1-book.vercel.app` | Site to scrape |
| `COLLECTION_NAME` | `physical-ai-book` | Qdrant collection name |
| `CHUNK_SIZE` | `800` | Target chunk size in characters |
| `CHUNK_OVERLAP` | `100` | Overlap between chunks |
| `BATCH_SIZE` | `96` | Embeddings per API call |

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success - all pages processed |
| 1 | Partial success - some pages failed, others succeeded |
| 2 | Configuration error - missing env vars |
| 3 | Network error - cannot reach target site or APIs |
| 4 | Fatal error - unexpected failure |

## Output Format

### Standard Output (stdout)

Progress and summary information:

```
Starting RAG Data Pipeline...
Fetching sitemap...
Found 22 content pages
Processing: [====================] 22/22 pages
Embedding:  [====================] 3/3 batches
Upserting:  [====================] 87/87 vectors

Pipeline Complete!
┌─────────────────────────────────────┐
│ Pages scraped:    22                │
│ Chunks created:   87                │
│ Vectors stored:   87                │
│ Errors:           0                 │
│ Duration:         45.3s             │
└─────────────────────────────────────┘
```

### Verbose Mode (--verbose)

Additional per-page details:

```
[INFO] Processing /docs/intro
[INFO]   Title: Introduction to Physical AI
[INFO]   Module: intro
[INFO]   Text length: 2,450 chars
[INFO]   Chunks: 4
[DEBUG] Chunk 0: 812 chars, ID: 550e8400-e29b-41d4-a716-446655440000
[DEBUG] Chunk 1: 798 chars, ID: 550e8400-e29b-41d4-a716-446655440001
...
```

### Error Output (stderr)

Errors and warnings:

```
[WARN] Skipping /docs/empty-page - no text content
[ERROR] Failed to scrape /docs/broken-link - 404 Not Found
[ERROR] Cohere rate limit hit - retrying in 60s (attempt 2/3)
```

## Validation Report

On completion, display module breakdown:

```
Module Summary:
  intro:      4 chunks
  ros2:       23 chunks
  simulation: 31 chunks
  isaac:      18 chunks
  vla:        11 chunks
  ─────────────────────
  Total:      87 chunks
```

## Idempotency

Running the pipeline multiple times is safe:
- Uses deterministic UUIDs (same content = same ID)
- Qdrant upsert updates existing vectors
- No duplicate vectors created

## Concurrency

- Sequential page scraping (polite crawling)
- Batched embedding requests (96 texts per batch)
- Batched vector upserts (100 points per batch)
