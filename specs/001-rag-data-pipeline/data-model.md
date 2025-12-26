# Data Model: RAG Data Pipeline

**Feature**: 001-rag-data-pipeline
**Date**: 2025-12-26

## Entities

### Page

Represents a scraped page from the Docusaurus site.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| url | string | Absolute URL of the page | Must be valid HTTPS URL |
| title | string | Page title from `<title>` or `<h1>` | Non-empty, max 200 chars |
| module_name | string | Module identifier | One of: intro, ros2, simulation, isaac, vla |
| raw_html | string | Original HTML content | Non-empty |
| extracted_text | string | Clean text content | Non-empty after extraction |
| scraped_at | datetime | Timestamp of scraping | ISO 8601 format |

**State Transitions**: None (stateless extraction)

**Relationships**:
- One Page → Many Chunks

---

### Chunk

A text segment derived from a page, ready for embedding.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| id | UUID | Deterministic ID (UUID5 from url + chunk_index) | Valid UUID |
| text | string | Chunk content | 400-1000 chars (target 800) |
| chunk_index | int | Position within page | >= 0 |
| char_count | int | Character count | > 0 |
| source_url | string | Parent page URL | Valid URL |
| page_title | string | Parent page title | Non-empty |
| module_name | string | Module identifier | One of: intro, ros2, simulation, isaac, vla |

**State Transitions**: None (derived entity)

**Relationships**:
- Many Chunks → One Page
- One Chunk → One Vector

---

### Vector (Qdrant Point)

An embedded chunk stored in Qdrant vector database.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| id | UUID | Same as Chunk.id | Valid UUID |
| vector | float[1024] | Cohere embedding | 1024 dimensions |
| payload.page_title | string | Page title for display | Non-empty |
| payload.page_url | string | Source URL for linking | Valid URL |
| payload.module_name | string | Module for filtering | One of: intro, ros2, simulation, isaac, vla |
| payload.chunk_index | int | Position for ordering | >= 0 |
| payload.text | string | Original text for display | Non-empty |

**State Transitions**:
- Created (first upsert)
- Updated (subsequent upsert with same ID)
- Deleted (explicit removal)

**Relationships**:
- One Vector ↔ One Chunk (1:1 via shared ID)

---

### Module

Logical grouping of content (enum/constant).

| Value | Description | URL Pattern |
|-------|-------------|-------------|
| intro | Introduction/overview | `/docs/intro` |
| ros2 | ROS 2 Fundamentals (Module 1) | `/docs/module-1/*` |
| simulation | Simulation & Digital Twins (Module 2) | `/docs/module-2/*` |
| isaac | NVIDIA Isaac Platform (Module 3) | `/docs/module-3/*` |
| vla | Vision-Language-Action (Module 4) | `/docs/module-4/*` |

---

## Qdrant Collection Schema

**Collection Name**: `physical-ai-book`

```json
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "payload_schema": {
    "page_title": { "type": "keyword" },
    "page_url": { "type": "keyword" },
    "module_name": { "type": "keyword", "indexed": true },
    "chunk_index": { "type": "integer" },
    "text": { "type": "text" }
  }
}
```

**Indexed Fields**:
- `module_name`: Enables filtering by module
- `page_url`: Enables filtering by source page

---

## Processing Pipeline Flow

```
[Sitemap] → [Page URLs] → [Scraper] → [Pages]
                                         ↓
[Pages] → [Chunker] → [Chunks] → [Embedder] → [Vectors]
                                                  ↓
                                            [Qdrant Upsert]
```

## ID Generation

Deterministic UUID5 ensures same chunk always gets same ID:

```python
import uuid

def generate_chunk_id(page_url: str, chunk_index: int) -> str:
    """Generate deterministic UUID for chunk deduplication."""
    name = f"{page_url}::{chunk_index}"
    return str(uuid.uuid5(uuid.NAMESPACE_URL, name))
```

This enables:
- Upsert behavior (update existing, insert new)
- No duplicate vectors on re-run
- Traceable ID from source data
