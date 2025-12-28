"""
Pytest configuration and fixtures for FastAPI backend tests.
"""
import pytest
from fastapi.testclient import TestClient


@pytest.fixture
def test_client():
    """
    Provides a FastAPI TestClient instance for testing API endpoints.

    Usage:
        def test_endpoint(test_client):
            response = test_client.get("/health")
            assert response.status_code == 200
    """
    # Import the FastAPI app from api.py
    import sys
    from pathlib import Path
    # Add parent directory to path to allow importing api module
    backend_dir = Path(__file__).parent.parent
    if str(backend_dir) not in sys.path:
        sys.path.insert(0, str(backend_dir))

    from api import app
    return TestClient(app)


@pytest.fixture
def valid_chat_request():
    """
    Provides a valid ChatRequest payload for testing.
    """
    return {
        "query": "What is ROS 2?",
        "top_k": 5
    }


@pytest.fixture
def chat_request_with_selected_text():
    """
    Provides a ChatRequest with selected_text for contextual queries.
    """
    return {
        "query": "Explain this code",
        "selected_text": "ros2 run demo_nodes_cpp talker",
        "top_k": 5
    }


@pytest.fixture
def invalid_empty_query():
    """
    Provides an invalid ChatRequest with empty query.
    """
    return {
        "query": "",
        "top_k": 5
    }


@pytest.fixture
def invalid_long_query():
    """
    Provides an invalid ChatRequest with query exceeding 2000 characters.
    """
    return {
        "query": "a" * 2001,
        "top_k": 5
    }
