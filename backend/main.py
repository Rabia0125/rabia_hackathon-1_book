"""
RAG Data Pipeline for Physical AI & Robotics Book

Scrapes content from Docusaurus site, generates embeddings with Cohere,
and stores vectors in Qdrant Cloud for RAG retrieval.
"""

import os
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional

import cohere
import httpx
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class Config:
    """Pipeline configuration loaded from environment."""
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    target_url: str = "https://rabia-hackathon-1-book.vercel.app"
    collection_name: str = "physical-ai-book"
    chunk_size: int = 800
    chunk_overlap: int = 100
    batch_size: int = 96


def load_config() -> Config:
    """Load configuration from .env file and environment variables."""
    load_dotenv()

    return Config(
        cohere_api_key=os.getenv("COHERE_API_KEY", ""),
        qdrant_url=os.getenv("QDRANT_URL", ""),
        qdrant_api_key=os.getenv("QDRANT_API_KEY", ""),
        target_url=os.getenv("TARGET_URL", "https://rabia-hackathon-1-book.vercel.app"),
        collection_name=os.getenv("COLLECTION_NAME", "physical-ai-book"),
        chunk_size=int(os.getenv("CHUNK_SIZE", "800")),
        chunk_overlap=int(os.getenv("CHUNK_OVERLAP", "100")),
        batch_size=int(os.getenv("BATCH_SIZE", "96")),
    )


def validate_config(config: Config) -> None:
    """Validate required configuration, fail fast with clear errors."""
    errors = []

    if not config.cohere_api_key:
        errors.append("COHERE_API_KEY is required but not set")
    if not config.qdrant_url:
        errors.append("QDRANT_URL is required but not set")
    if not config.qdrant_api_key:
        errors.append("QDRANT_API_KEY is required but not set")
    if not config.target_url:
        errors.append("TARGET_URL is required but not set")

    if errors:
        print("Configuration errors:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        print("\nPlease set required environment variables in .env file.", file=sys.stderr)
        print("See .env.example for template.", file=sys.stderr)
        sys.exit(2)


# =============================================================================
# Data Models
# =============================================================================

@dataclass
class Page:
    """Represents a scraped page from the Docusaurus site."""
    url: str
    title: str
    module_name: str
    extracted_text: str


@dataclass
class Chunk:
    """A text segment derived from a page, ready for embedding."""
    id: str
    text: str
    chunk_index: int
    char_count: int
    source_url: str
    page_title: str
    module_name: str


@dataclass
class PipelineStats:
    """Track pipeline processing statistics."""
    pages_scraped: int = 0
    pages_failed: int = 0
    chunks_created: int = 0
    vectors_stored: int = 0
    errors: list = field(default_factory=list)
    by_module: dict = field(default_factory=dict)

    def add_page(self, module: str, chunk_count: int) -> None:
        """Record a successfully processed page."""
        self.pages_scraped += 1
        self.chunks_created += chunk_count
        if module not in self.by_module:
            self.by_module[module] = {"pages": 0, "chunks": 0}
        self.by_module[module]["pages"] += 1
        self.by_module[module]["chunks"] += chunk_count

    def add_error(self, url: str, error: str) -> None:
        """Record a page processing error."""
        self.pages_failed += 1
        self.errors.append({"url": url, "error": error})


# =============================================================================
# Web Scraping
# =============================================================================

def fetch_sitemap(base_url: str) -> list[str]:
    """Fetch and parse sitemap.xml, return list of content URLs (filter out /tags/)."""
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    print(f"Fetching sitemap from {sitemap_url}")

    response = httpx.get(sitemap_url, timeout=30.0)
    response.raise_for_status()

    # Parse XML
    root = ET.fromstring(response.content)

    # Handle namespace
    ns = {"sm": "http://www.sitemaps.org/schemas/sitemap/0.9"}

    urls = []
    for url_elem in root.findall(".//sm:url/sm:loc", ns):
        url = url_elem.text
        if url:
            # Filter out tag pages and keep only content pages
            if "/tags/" not in url and "/tags" not in url.split("/")[-1]:
                urls.append(url)

    print(f"Found {len(urls)} content URLs (filtered from sitemap)")
    return urls


def crawl_homepage(base_url: str) -> list[str]:
    """Fallback: crawl homepage to discover documentation links."""
    print(f"Crawling homepage for documentation links...")

    response = httpx.get(base_url, timeout=30.0)
    response.raise_for_status()

    soup = BeautifulSoup(response.content, "html.parser")

    # Find all internal links that look like documentation
    urls = set()
    base = base_url.rstrip("/")

    for link in soup.find_all("a", href=True):
        href = link["href"]
        # Only internal doc links
        if href.startswith("/docs/") or href.startswith("/book/"):
            # Skip tags
            if "/tags" in href:
                continue
            full_url = f"{base}{href}"
            urls.add(full_url)
        elif href.startswith(base) and ("/docs/" in href or "/book/" in href):
            if "/tags" not in href:
                urls.add(href)

    url_list = sorted(urls)
    print(f"Found {len(url_list)} documentation links from homepage")
    return url_list


def extract_module(url: str) -> str:
    """Derive module name from URL path."""
    if "/docs/module-1" in url or "/module-1/" in url:
        return "ros2"
    elif "/docs/module-2" in url or "/module-2/" in url:
        return "simulation"
    elif "/docs/module-3" in url or "/module-3/" in url:
        return "isaac"
    elif "/docs/module-4" in url or "/module-4/" in url:
        return "vla"
    elif "/docs/intro" in url:
        return "intro"
    else:
        return "general"


def scrape_page(url: str, delay: float = 0.5, max_retries: int = 3) -> Optional[Page]:
    """
    Scrape a single page with retry logic.

    Args:
        url: Page URL to scrape
        delay: Delay between requests for polite crawling
        max_retries: Maximum retry attempts for network errors

    Returns:
        Page object or None if scraping failed
    """
    time.sleep(delay)  # Polite crawling

    for attempt in range(max_retries):
        try:
            response = httpx.get(url, timeout=30.0, follow_redirects=True)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, "html.parser")

            # Extract title
            title_tag = soup.find("title")
            title = title_tag.get_text(strip=True) if title_tag else url.split("/")[-1]

            # Remove non-content elements
            for elem in soup.find_all(["nav", "footer", "header", "aside", "script", "style"]):
                elem.decompose()

            # Extract main content (Docusaurus uses <article> or <main>)
            content = soup.find("article") or soup.find("main") or soup.find("body")

            if content:
                # Get text, preserving some structure
                text = content.get_text(separator="\n", strip=True)
                # Clean up excessive whitespace
                lines = [line.strip() for line in text.split("\n") if line.strip()]
                text = "\n".join(lines)
            else:
                text = ""

            if not text:
                return None

            return Page(
                url=url,
                title=title,
                module_name=extract_module(url),
                extracted_text=text
            )

        except httpx.HTTPStatusError as e:
            if attempt == max_retries - 1:
                raise
            wait_time = 2 ** attempt  # Exponential backoff
            print(f"  Retry {attempt + 1}/{max_retries} for {url} after {wait_time}s (HTTP {e.response.status_code})")
            time.sleep(wait_time)
        except httpx.RequestError as e:
            if attempt == max_retries - 1:
                raise
            wait_time = 2 ** attempt
            print(f"  Retry {attempt + 1}/{max_retries} for {url} after {wait_time}s ({type(e).__name__})")
            time.sleep(wait_time)

    return None


# =============================================================================
# Text Processing
# =============================================================================

def chunk_text(text: str, size: int = 800, overlap: int = 100) -> list[str]:
    """
    Split text into chunks with overlap.

    Args:
        text: Text to chunk
        size: Target chunk size in characters
        overlap: Overlap between chunks

    Returns:
        List of text chunks
    """
    if len(text) <= size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + size

        # Try to break at paragraph boundary
        if end < len(text):
            # Look for paragraph break near the end
            para_break = text.rfind("\n\n", start + size // 2, end)
            if para_break > start:
                end = para_break
            else:
                # Look for sentence break
                for sep in [". ", "! ", "? ", "\n"]:
                    sent_break = text.rfind(sep, start + size // 2, end)
                    if sent_break > start:
                        end = sent_break + len(sep)
                        break

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        # Move start with overlap
        start = end - overlap if end < len(text) else len(text)

    return chunks


def generate_chunk_id(url: str, chunk_index: int) -> str:
    """Generate deterministic UUID5 for chunk deduplication."""
    name = f"{url}::{chunk_index}"
    return str(uuid.uuid5(uuid.NAMESPACE_URL, name))


# =============================================================================
# Embedding Generation
# =============================================================================

def generate_embeddings(
    chunks: list[Chunk],
    cohere_client: cohere.Client,
    batch_size: int = 96
) -> list[list[float]]:
    """
    Generate embeddings using Cohere batch API.

    Args:
        chunks: List of chunks to embed
        cohere_client: Cohere client instance
        batch_size: Texts per API call (max 96)

    Returns:
        List of embedding vectors
    """
    all_embeddings = []
    texts = [c.text for c in chunks]

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        batch_num = i // batch_size + 1
        total_batches = (len(texts) + batch_size - 1) // batch_size
        print(f"  Generating embeddings (batch {batch_num}/{total_batches})...")

        try:
            response = cohere_client.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            all_embeddings.extend(response.embeddings)
        except cohere.TooManyRequestsError:
            # Rate limit hit - wait and retry
            print("  Rate limit hit, waiting 60s...")
            time.sleep(60)
            response = cohere_client.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            all_embeddings.extend(response.embeddings)

    return all_embeddings


# =============================================================================
# Vector Storage
# =============================================================================

def init_qdrant_collection(client: QdrantClient, collection_name: str) -> None:
    """Create Qdrant collection if it doesn't exist."""
    collections = client.get_collections().collections
    exists = any(c.name == collection_name for c in collections)

    if not exists:
        print(f"Creating collection '{collection_name}'...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
        )
    else:
        print(f"Collection '{collection_name}' already exists")


def upsert_vectors(
    client: QdrantClient,
    collection_name: str,
    chunks: list[Chunk],
    embeddings: list[list[float]],
    batch_size: int = 100
) -> int:
    """
    Upsert vectors to Qdrant with metadata payload.

    Args:
        client: Qdrant client
        collection_name: Target collection
        chunks: Chunk metadata
        embeddings: Embedding vectors
        batch_size: Points per upsert call

    Returns:
        Number of vectors upserted
    """
    points = [
        PointStruct(
            id=chunk.id,
            vector=embedding,
            payload={
                "page_title": chunk.page_title,
                "page_url": chunk.source_url,
                "module_name": chunk.module_name,
                "chunk_index": chunk.chunk_index,
                "text": chunk.text
            }
        )
        for chunk, embedding in zip(chunks, embeddings)
    ]

    # Batch upsert
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(collection_name=collection_name, points=batch)

    return len(points)


def cleanup_stale_vectors(
    client: QdrantClient,
    collection_name: str,
    current_ids: set[str]
) -> int:
    """
    Delete vectors not in current batch (stale content).

    Args:
        client: Qdrant client
        collection_name: Target collection
        current_ids: Set of chunk IDs from current run

    Returns:
        Number of vectors deleted
    """
    # Get all existing point IDs
    scroll_result = client.scroll(
        collection_name=collection_name,
        limit=10000,
        with_payload=False,
        with_vectors=False
    )

    existing_ids = {str(point.id) for point in scroll_result[0]}
    stale_ids = existing_ids - current_ids

    if stale_ids:
        print(f"  Removing {len(stale_ids)} stale vectors...")
        client.delete(
            collection_name=collection_name,
            points_selector=list(stale_ids)
        )

    return len(stale_ids)


# =============================================================================
# Reporting
# =============================================================================

def print_summary(stats: PipelineStats) -> None:
    """Print formatted summary report."""
    print("\n" + "=" * 50)
    print("Pipeline Complete!")
    print("=" * 50)
    print(f"  Pages scraped:    {stats.pages_scraped}")
    print(f"  Pages failed:     {stats.pages_failed}")
    print(f"  Chunks created:   {stats.chunks_created}")
    print(f"  Vectors stored:   {stats.vectors_stored}")
    print("-" * 50)
    print("Module Breakdown:")
    for module, data in sorted(stats.by_module.items()):
        print(f"  {module:12} {data['pages']:3} pages, {data['chunks']:4} chunks")

    if stats.errors:
        print("-" * 50)
        print("Errors:")
        for err in stats.errors[:5]:  # Show first 5 errors
            print(f"  - {err['url']}: {err['error']}")
        if len(stats.errors) > 5:
            print(f"  ... and {len(stats.errors) - 5} more errors")
    print("=" * 50)


# =============================================================================
# Main Pipeline
# =============================================================================

def main() -> None:
    """Main pipeline entry point."""
    print("Starting RAG Data Pipeline...")
    print("-" * 50)

    # Load and validate config
    config = load_config()
    validate_config(config)

    # Initialize clients
    cohere_client = cohere.Client(api_key=config.cohere_api_key)
    qdrant_client = QdrantClient(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key
    )

    # Initialize collection
    init_qdrant_collection(qdrant_client, config.collection_name)

    # Initialize stats
    stats = PipelineStats()
    all_chunks: list[Chunk] = []
    all_chunk_ids: set[str] = set()

    # Fetch sitemap
    try:
        urls = fetch_sitemap(config.target_url)
    except Exception as e:
        print(f"Failed to fetch sitemap: {e}", file=sys.stderr)
        sys.exit(3)

    # Process each page
    print(f"\nProcessing {len(urls)} pages...")
    for i, url in enumerate(urls, 1):
        print(f"  [{i}/{len(urls)}] {url.split('/')[-1] or 'index'}...", end=" ")

        try:
            page = scrape_page(url, delay=0.5)

            if not page or not page.extracted_text:
                print("(no content)")
                continue

            # Chunk the text
            text_chunks = chunk_text(
                page.extracted_text,
                size=config.chunk_size,
                overlap=config.chunk_overlap
            )

            # Create Chunk objects
            page_chunks = []
            for idx, text in enumerate(text_chunks):
                chunk_id = generate_chunk_id(page.url, idx)
                chunk = Chunk(
                    id=chunk_id,
                    text=text,
                    chunk_index=idx,
                    char_count=len(text),
                    source_url=page.url,
                    page_title=page.title,
                    module_name=page.module_name
                )
                page_chunks.append(chunk)
                all_chunk_ids.add(chunk_id)

            all_chunks.extend(page_chunks)
            stats.add_page(page.module_name, len(page_chunks))
            print(f"{len(page_chunks)} chunks")

        except Exception as e:
            print(f"ERROR: {e}")
            stats.add_error(url, str(e))

    # Generate embeddings
    if all_chunks:
        print(f"\nGenerating embeddings for {len(all_chunks)} chunks...")
        embeddings = generate_embeddings(
            all_chunks,
            cohere_client,
            batch_size=config.batch_size
        )

        # Upsert to Qdrant
        print(f"\nUpserting {len(all_chunks)} vectors to Qdrant...")
        stats.vectors_stored = upsert_vectors(
            qdrant_client,
            config.collection_name,
            all_chunks,
            embeddings
        )

        # Cleanup stale vectors
        deleted = cleanup_stale_vectors(
            qdrant_client,
            config.collection_name,
            all_chunk_ids
        )
        if deleted:
            print(f"  Cleaned up {deleted} stale vectors")

    # Print summary
    print_summary(stats)


if __name__ == "__main__":
    main()
