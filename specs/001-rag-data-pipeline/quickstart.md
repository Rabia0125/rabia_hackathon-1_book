# Quickstart: RAG Data Pipeline

**Feature**: 001-rag-data-pipeline
**Date**: 2025-12-26

## Prerequisites

1. **Python 3.11+** installed
2. **UV package manager** installed ([installation guide](https://docs.astral.sh/uv/getting-started/installation/))
3. **Cohere API Key** ([sign up](https://dashboard.cohere.com/))
4. **Qdrant Cloud Account** ([free tier](https://cloud.qdrant.io/))

## Setup

### 1. Clone and Navigate

```bash
cd My-Book
```

### 2. Initialize Backend Project

```bash
uv init backend
cd backend
```

### 3. Install Dependencies

```bash
uv add cohere qdrant-client python-dotenv httpx beautifulsoup4
```

### 4. Configure Environment

Create `backend/.env`:

```env
# Cohere API
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# Pipeline Config
TARGET_URL=https://rabia-hackathon-1-book.vercel.app
COLLECTION_NAME=physical-ai-book
CHUNK_SIZE=800
CHUNK_OVERLAP=100
BATCH_SIZE=96
```

### 5. Get Your API Keys

**Cohere**:
1. Go to [Cohere Dashboard](https://dashboard.cohere.com/)
2. Navigate to API Keys
3. Create a new key or copy existing

**Qdrant Cloud**:
1. Go to [Qdrant Cloud Console](https://cloud.qdrant.io/)
2. Create a free cluster (1GB RAM)
3. Copy the cluster URL and API key from cluster details

## Running the Pipeline

### Full Ingestion

```bash
cd backend
uv run python main.py
```

Expected output:
```
Starting RAG Data Pipeline...
Fetching sitemap from https://rabia-hackathon-1-book.vercel.app/sitemap.xml
Found 22 content pages (filtered from 77 total URLs)
Processing pages...
  [1/22] /docs/intro - 3 chunks
  [2/22] /docs/module-1/fundamentals - 5 chunks
  ...
Generating embeddings (batch 1/3)...
Upserting vectors to Qdrant...

Pipeline Complete!
- Pages scraped: 22
- Chunks created: 87
- Vectors stored: 87
- Errors: 0
```

### Re-indexing (Update Content)

Simply run the same command again:

```bash
uv run python main.py
```

The pipeline uses upsert, so existing vectors are updated (not duplicated).

## Verification

### Check Qdrant Collection

```python
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Get collection info
info = client.get_collection("physical-ai-book")
print(f"Vectors count: {info.points_count}")

# Sample query by module
results = client.scroll(
    collection_name="physical-ai-book",
    scroll_filter={"must": [{"key": "module_name", "match": {"value": "ros2"}}]},
    limit=5
)
for point in results[0]:
    print(f"- {point.payload['page_title']}: chunk {point.payload['chunk_index']}")
```

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| `COHERE_API_KEY not set` | Check `.env` file exists and has correct key |
| `Connection refused (Qdrant)` | Verify QDRANT_URL includes port `:6333` |
| `Rate limit exceeded` | Wait 60s, then retry; consider using production API key |
| `No pages found` | Check TARGET_URL is accessible and has sitemap.xml |

### Debug Mode

Add verbose logging:

```bash
uv run python main.py --verbose
```

## Project Structure

```
backend/
├── .env              # API keys (not committed)
├── .env.example      # Template for .env
├── pyproject.toml    # UV project config
├── uv.lock           # Locked dependencies
└── main.py           # Pipeline entry point
```

## Next Steps

After successful ingestion:
- **Spec 2**: Implement retrieval/search functionality
- **Spec 3**: Integrate with AI agent for Q&A
- **Spec 4**: Build frontend chat interface
