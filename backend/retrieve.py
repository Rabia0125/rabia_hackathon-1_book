"""
RAG Retrieval Pipeline for Physical AI & Robotics Book

Queries the Qdrant vector database to find relevant text chunks for user queries.
Supports filtering by module/URL and includes a validation test suite.

Usage:
    python retrieve.py "query text" [options]
    python retrieve.py --validate
"""

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue


# =============================================================================
# Configuration (T001-T004)
# =============================================================================

@dataclass
class Config:
    """Pipeline configuration loaded from environment."""
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    collection_name: str = "physical-ai-book"


def load_config() -> Config:
    """Load configuration from .env file and environment variables."""
    load_dotenv()

    return Config(
        cohere_api_key=os.getenv("COHERE_API_KEY", ""),
        qdrant_url=os.getenv("QDRANT_URL", ""),
        qdrant_api_key=os.getenv("QDRANT_API_KEY", ""),
        collection_name=os.getenv("COLLECTION_NAME", "physical-ai-book"),
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

    if errors:
        print("[ERROR] Configuration error:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        print("\nPlease set required environment variables in .env file.", file=sys.stderr)
        sys.exit(2)


# =============================================================================
# Data Models (T005-T006)
# =============================================================================

@dataclass
class SearchResult:
    """A single retrieved chunk with its metadata and relevance score."""
    text: str
    score: float
    page_title: str
    page_url: str
    module_name: str
    chunk_index: int


@dataclass
class RetrievalResponse:
    """Complete response for a search query."""
    query: str
    results: list[SearchResult]
    total_found: int
    query_time_ms: float
    filters_applied: dict = field(default_factory=dict)


# =============================================================================
# Validation Data Models (T019-T020)
# =============================================================================

@dataclass
class ValidationResult:
    """Result of a single test query in validation mode."""
    query: str
    expected_module: str
    actual_module: Optional[str]
    top_score: Optional[float]
    passed: bool


@dataclass
class ValidationReport:
    """Summary of validation test suite execution."""
    total_tests: int
    passed: int
    failed: int
    accuracy: float
    results: list[ValidationResult]
    execution_time_ms: float


# =============================================================================
# Test Queries (T018)
# =============================================================================

TEST_QUERIES = [
    ("What is ROS 2 and how do nodes communicate?", "ros2"),
    ("How do I set up Gazebo for robot simulation?", "simulation"),
    ("What is NVIDIA Isaac Sim and Isaac ROS?", "isaac"),
    ("How do VLA models work for robot control?", "vla"),
    ("What topics does this book cover?", "intro"),
]


# =============================================================================
# Core Retrieval Functions (T007-T010)
# =============================================================================

def embed_query(text: str, cohere_client: cohere.Client, max_retries: int = 3) -> list[float]:
    """
    Generate embedding for a query using Cohere API.

    Args:
        text: Query text to embed
        cohere_client: Cohere client instance
        max_retries: Maximum retry attempts for rate limits

    Returns:
        1024-dimensional embedding vector
    """
    for attempt in range(max_retries):
        try:
            response = cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            return response.embeddings[0]
        except cohere.TooManyRequestsError:
            if attempt == max_retries - 1:
                raise
            wait_time = 2 ** attempt
            print(f"[WARN] Rate limit hit, waiting {wait_time}s...", file=sys.stderr)
            time.sleep(wait_time)

    return []


def build_filter(module: Optional[str] = None, url: Optional[str] = None) -> Optional[Filter]:
    """
    Build Qdrant filter from module and URL constraints.

    Args:
        module: Module name to filter by (intro, ros2, simulation, isaac, vla)
        url: Page URL to filter by (exact match)

    Returns:
        Qdrant Filter object or None if no filters specified
    """
    conditions = []

    if module:
        conditions.append(
            FieldCondition(
                key="module_name",
                match=MatchValue(value=module)
            )
        )

    if url:
        conditions.append(
            FieldCondition(
                key="page_url",
                match=MatchValue(value=url)
            )
        )

    if not conditions:
        return None

    return Filter(must=conditions)


def search_qdrant(
    query_vector: list[float],
    client: QdrantClient,
    collection: str,
    top_k: int = 5,
    query_filter: Optional[Filter] = None
) -> list:
    """
    Search Qdrant for similar vectors.

    Args:
        query_vector: Query embedding vector
        client: Qdrant client
        collection: Collection name
        top_k: Number of results to return
        query_filter: Optional filter conditions

    Returns:
        List of ScoredPoint results
    """
    try:
        results = client.query_points(
            collection_name=collection,
            query=query_vector,
            limit=top_k,
            with_payload=True,
            query_filter=query_filter
        )
        return results.points
    except Exception as e:
        print(f"[ERROR] Qdrant search failed: {e}", file=sys.stderr)
        raise


def format_results(qdrant_results: list) -> list[SearchResult]:
    """
    Convert Qdrant ScoredPoints to SearchResult objects.

    Args:
        qdrant_results: List of ScoredPoint from Qdrant

    Returns:
        List of SearchResult objects
    """
    results = []
    for point in qdrant_results:
        payload = point.payload
        results.append(SearchResult(
            text=payload.get("text", ""),
            score=point.score,
            page_title=payload.get("page_title", ""),
            page_url=payload.get("page_url", ""),
            module_name=payload.get("module_name", ""),
            chunk_index=payload.get("chunk_index", 0)
        ))
    return results


def retrieve(
    query: str,
    config: Config,
    top_k: int = 5,
    module_filter: Optional[str] = None,
    url_filter: Optional[str] = None
) -> RetrievalResponse:
    """
    Execute full retrieval pipeline: embed → search → format.

    Args:
        query: Natural language query
        config: Configuration object
        top_k: Number of results to return
        module_filter: Optional module name filter
        url_filter: Optional URL filter

    Returns:
        RetrievalResponse with ranked results
    """
    start_time = time.time()

    # Initialize clients
    cohere_client = cohere.Client(api_key=config.cohere_api_key)
    qdrant_client = QdrantClient(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key
    )

    # Build filter
    query_filter = build_filter(module=module_filter, url=url_filter)

    # Embed query
    query_vector = embed_query(query, cohere_client)

    # Search Qdrant
    qdrant_results = search_qdrant(
        query_vector,
        qdrant_client,
        config.collection_name,
        top_k=top_k,
        query_filter=query_filter
    )

    # Format results
    results = format_results(qdrant_results)

    # Calculate time
    query_time_ms = (time.time() - start_time) * 1000

    # Build filters applied dict
    filters_applied = {}
    if module_filter:
        filters_applied["module"] = module_filter
    if url_filter:
        filters_applied["url"] = url_filter

    return RetrievalResponse(
        query=query,
        results=results,
        total_found=len(results),
        query_time_ms=query_time_ms,
        filters_applied=filters_applied
    )


# =============================================================================
# Output Formatting (T011, T017, T025-T027)
# =============================================================================

def print_results(response: RetrievalResponse, verbose: bool = False) -> None:
    """
    Print search results in standard text format.

    Args:
        response: RetrievalResponse object
        verbose: Include chunk text in output
    """
    print(f'Query: "{response.query}"')

    if response.filters_applied:
        filters_str = ", ".join(f"{k}={v}" for k, v in response.filters_applied.items())
        print(f"Filters: {filters_str}")

    print(f"Found {response.total_found} results in {response.query_time_ms:.0f}ms")
    print()

    if not response.results:
        print("No results found.")
        return

    for i, result in enumerate(response.results, 1):
        print(f"[{i}] Score: {result.score:.3f} | Module: {result.module_name}")
        print(f"    Title: {result.page_title}")
        print(f"    URL: {result.page_url}")
        if verbose:
            # Truncate text for display
            text_preview = result.text[:200] + "..." if len(result.text) > 200 else result.text
            print(f"    Text: \"{text_preview}\"")
        print()


def print_json(response: RetrievalResponse) -> None:
    """
    Print search results in JSON format.

    Args:
        response: RetrievalResponse object
    """
    output = {
        "query": response.query,
        "total_found": response.total_found,
        "query_time_ms": round(response.query_time_ms, 2),
        "filters": response.filters_applied,
        "results": [
            {
                "score": round(r.score, 3),
                "module_name": r.module_name,
                "page_title": r.page_title,
                "page_url": r.page_url,
                "chunk_index": r.chunk_index,
                "text": r.text
            }
            for r in response.results
        ]
    }
    print(json.dumps(output, indent=2))


# =============================================================================
# Validation Functions (T021-T022)
# =============================================================================

def run_validation(config: Config) -> ValidationReport:
    """
    Execute validation test suite against all modules.

    Args:
        config: Configuration object

    Returns:
        ValidationReport with accuracy metrics
    """
    start_time = time.time()
    results = []
    passed = 0
    failed = 0

    for query, expected_module in TEST_QUERIES:
        try:
            response = retrieve(query, config, top_k=1)

            if response.results:
                actual_module = response.results[0].module_name
                top_score = response.results[0].score
                test_passed = actual_module == expected_module
            else:
                actual_module = None
                top_score = None
                test_passed = False

            if test_passed:
                passed += 1
            else:
                failed += 1

            results.append(ValidationResult(
                query=query,
                expected_module=expected_module,
                actual_module=actual_module,
                top_score=top_score,
                passed=test_passed
            ))

        except Exception as e:
            failed += 1
            results.append(ValidationResult(
                query=query,
                expected_module=expected_module,
                actual_module=None,
                top_score=None,
                passed=False
            ))

    total = len(TEST_QUERIES)
    accuracy = (passed / total * 100) if total > 0 else 0
    execution_time_ms = (time.time() - start_time) * 1000

    return ValidationReport(
        total_tests=total,
        passed=passed,
        failed=failed,
        accuracy=accuracy,
        results=results,
        execution_time_ms=execution_time_ms
    )


def print_validation_report(report: ValidationReport, verbose: bool = False) -> None:
    """
    Print validation report in formatted output.

    Args:
        report: ValidationReport object
        verbose: Show additional details
    """
    print("Running Retrieval Validation Suite...")
    print()

    for result in report.results:
        status = "[PASS]" if result.passed else "[FAIL]"
        print(f'{status} "{result.query}"')

        if result.actual_module:
            score_str = f"{result.top_score:.3f}" if result.top_score else "N/A"
            print(f"       Expected: {result.expected_module} | Actual: {result.actual_module} | Score: {score_str}")
        else:
            print(f"       Expected: {result.expected_module} | Actual: (no results)")
        print()

    print("=" * 50)
    print("Validation Complete!")
    print("=" * 50)
    print(f"  Total Tests:  {report.total_tests}")
    print(f"  Passed:       {report.passed}")
    print(f"  Failed:       {report.failed}")
    print(f"  Accuracy:     {report.accuracy:.1f}%")
    print(f"  Duration:     {report.execution_time_ms / 1000:.2f}s")
    print("=" * 50)


# =============================================================================
# Main Entry Point (T012, T016, T023-T024, T028, T031)
# =============================================================================

def main() -> None:
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="RAG Retrieval Pipeline for Physical AI & Robotics Book",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python retrieve.py "What is ROS 2?"
  python retrieve.py "sensors" --module simulation
  python retrieve.py "robot navigation" --top-k 10
  python retrieve.py --validate
        """
    )

    parser.add_argument(
        "query",
        nargs="?",
        help="Natural language query (required unless --validate is used)"
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=5,
        help="Number of results to return (1-100, default: 5)"
    )
    parser.add_argument(
        "--module",
        choices=["intro", "ros2", "simulation", "isaac", "vla"],
        help="Filter by module name"
    )
    parser.add_argument(
        "--url",
        help="Filter by page URL (exact match)"
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Run validation test suite instead of query"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show detailed output including chunk text"
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output results as JSON"
    )

    args = parser.parse_args()

    # Validate top-k range
    if args.top_k < 1 or args.top_k > 100:
        print("[ERROR] Invalid argument: --top-k must be between 1 and 100", file=sys.stderr)
        sys.exit(4)

    # Load and validate config
    config = load_config()
    validate_config(config)

    # Run validation mode
    if args.validate:
        try:
            report = run_validation(config)
            print_validation_report(report, verbose=args.verbose)

            # Exit code based on accuracy threshold (80%)
            if report.accuracy >= 80:
                sys.exit(0)
            else:
                sys.exit(5)

        except Exception as e:
            print(f"[ERROR] Validation failed: {e}", file=sys.stderr)
            sys.exit(3)

    # Query mode - require query argument
    if not args.query:
        print("[ERROR] Query cannot be empty", file=sys.stderr)
        sys.exit(4)

    if not args.query.strip():
        print("[ERROR] Query cannot be empty", file=sys.stderr)
        sys.exit(4)

    # Execute retrieval
    try:
        response = retrieve(
            query=args.query,
            config=config,
            top_k=args.top_k,
            module_filter=args.module,
            url_filter=args.url
        )

        # Output results
        if args.json:
            print_json(response)
        else:
            print_results(response, verbose=args.verbose)

        # Exit code
        if response.total_found == 0:
            sys.exit(1)  # No results found
        else:
            sys.exit(0)  # Success

    except Exception as e:
        print(f"[ERROR] Retrieval failed: {e}", file=sys.stderr)
        sys.exit(3)


if __name__ == "__main__":
    main()
