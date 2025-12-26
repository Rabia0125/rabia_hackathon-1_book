# Research: RAG Data Pipeline

**Feature**: 001-rag-data-pipeline
**Date**: 2025-12-26

## Research Findings

### 1. Cohere Embed API

**Decision**: Use Cohere embed-v3-english model with batch size of 96 texts per call

**Rationale**:
- Embed v3 models are well-documented and stable for production use
- Maximum batch size of 96 texts per API call minimizes requests
- Each text input must be under 512 tokens (console) or 128K tokens total per batch (API)
- Output dimensions: 1024 for embed-v3-english, 1536 for embed-v4

**Alternatives Considered**:
- OpenAI embeddings: Higher cost, similar quality
- Local models (sentence-transformers): Requires GPU, more setup complexity
- Cohere embed-v4: Newer but overkill for text-only use case

**Implementation Notes**:
- Use `cohere` Python SDK
- Set `input_type="search_document"` for indexing content
- Handle rate limits with exponential backoff (free tier has lower limits)
- Production API keys have higher rate limits than trial keys

**Sources**:
- [Embed API Reference](https://docs.cohere.com/v1/reference/embed)
- [Rate Limits](https://docs.cohere.com/docs/rate-limits)
- [Batch Embedding Jobs](https://docs.cohere.com/docs/embed-jobs-api)

---

### 2. Qdrant Cloud Vector Database

**Decision**: Use Qdrant Cloud Free Tier with Python client, UUID-based point IDs

**Rationale**:
- Free tier provides 1GB RAM, sufficient for book content
- Upsert operation handles both insert and update (enables re-indexing)
- Python client supports batch operations via `upsert()` method
- Metadata stored as "payload" alongside vectors

**Alternatives Considered**:
- Pinecone: Good but free tier is more limited
- Weaviate Cloud: More complex schema requirements
- Self-hosted Qdrant: Requires infrastructure management

**Implementation Notes**:
- Connect with `QdrantClient(url=..., api_key=...)`
- Create collection with vector dimension matching Cohere output (1024)
- Use `PointStruct` for upserting with payload (metadata)
- Generate deterministic IDs from URL+chunk_index for deduplication
- Batch upserts in chunks of ~100 points for optimal performance

**Sources**:
- [Qdrant Python Client](https://github.com/qdrant/qdrant-client)
- [Quickstart Guide](https://python-client.qdrant.tech/quickstart)
- [PyPI Package](https://pypi.org/project/qdrant-client/)

---

### 3. UV Package Manager

**Decision**: Use UV for project initialization and dependency management

**Rationale**:
- Fast dependency installation (10-100x faster than pip)
- Automatic virtual environment management
- Built-in lockfile support for reproducibility
- Single tool replaces pip, virtualenv, pyenv

**Alternatives Considered**:
- Poetry: Slower, more complex configuration
- pip + venv: Manual virtual environment management
- Conda: Heavier, designed for data science workflows

**Implementation Notes**:
- Initialize with `uv init backend`
- Add dependencies with `uv add cohere qdrant-client python-dotenv httpx beautifulsoup4`
- Run scripts with `uv run python main.py`
- Lock dependencies with `uv lock`

**Sources**:
- [UV Documentation](https://docs.astral.sh/uv/)
- [GitHub Repository](https://github.com/astral-sh/uv)

---

### 4. Web Scraping Strategy

**Decision**: Use httpx + BeautifulSoup4 for Docusaurus site scraping with sitemap discovery

**Rationale**:
- Docusaurus generates sitemap.xml at root (verified: 77 URLs)
- httpx provides modern async HTTP with connection pooling
- BeautifulSoup4 handles HTML parsing and text extraction
- Sitemap-based discovery is more reliable than link crawling

**Alternatives Considered**:
- Scrapy: Overkill for static site scraping
- Selenium/Playwright: Not needed for server-rendered content
- requests: Lacks async support, less efficient

**Implementation Notes**:
- Parse sitemap.xml to get all page URLs
- Filter out tag pages (only index actual content pages)
- Extract text from `<article>` or main content div
- Strip navigation, footer, sidebar elements
- Add 0.5s delay between requests (polite crawling)

**Site Structure** (from sitemap analysis):
- Total URLs: 77
- Tag pages: ~55 (skip for indexing)
- Content pages: ~22 (index these)
- Modules: intro, module-1 (ROS 2), module-2 (Simulation), module-3 (Isaac), module-4 (VLA)

---

### 5. Text Chunking Strategy

**Decision**: Use character-based chunking with 800 char target, 100 char overlap

**Rationale**:
- 800 characters fits within Cohere's token limits
- 100 char overlap (~12.5%) preserves context across boundaries
- Paragraph-aware splitting maintains semantic coherence
- Simple implementation without external dependencies

**Alternatives Considered**:
- Token-based chunking: Requires tokenizer, more complex
- Semantic chunking: Expensive, requires additional API calls
- Fixed-size only: Loses context at chunk boundaries

**Implementation Notes**:
- Split on paragraph boundaries first (double newlines)
- Merge small paragraphs until reaching target size
- Split large paragraphs at sentence boundaries
- Add overlap by including end of previous chunk
- Track chunk_index per page for deterministic IDs

---

### 6. Deduplication Strategy

**Decision**: Use deterministic UUID5 based on URL + chunk_index

**Rationale**:
- UUID5 generates same ID for same input (namespace + name)
- URL + chunk_index uniquely identifies each chunk
- Qdrant upsert replaces existing points with same ID
- No need for external ID tracking database

**Implementation Notes**:
```python
import uuid
point_id = str(uuid.uuid5(uuid.NAMESPACE_URL, f"{page_url}::{chunk_index}"))
```
- Same chunk always gets same ID across runs
- Changed content replaces old vector (upsert behavior)
- Deleted pages need explicit cleanup (not automatic)

---

## Resolved Clarifications

| Original Unknown | Resolution |
|------------------|------------|
| Embedding model choice | Cohere embed-v3-english (1024 dimensions) |
| Batch size for embeddings | 96 texts per API call |
| Vector DB collection schema | Single collection "physical-ai-book" |
| Chunk size target | 800 chars with 100 char overlap |
| Page discovery method | sitemap.xml parsing |
| ID generation for dedup | UUID5 from URL + chunk_index |

## Technology Stack Summary

| Component | Choice | Version/Config |
|-----------|--------|----------------|
| Language | Python | 3.11+ |
| Package Manager | UV | Latest |
| HTTP Client | httpx | Async |
| HTML Parser | BeautifulSoup4 | html.parser |
| Embeddings | Cohere | embed-v3-english |
| Vector DB | Qdrant Cloud | Free Tier |
| Config | python-dotenv | .env file |
