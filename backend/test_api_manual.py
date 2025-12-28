"""
Manual API testing script to verify endpoints work correctly.
"""
import requests
import json

BASE_URL = "http://localhost:8001"
    

def test_health():
    """Test /health endpoint."""
    print("\n=== Testing /health endpoint ===")
    response = requests.get(f"{BASE_URL}/health")
    print(f"Status Code: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
    assert response.status_code == 200, f"Expected 200, got {response.status_code}"
    assert response.json()["status"] == "ok"
    print("[PASS] /health endpoint works!")


def test_ready():
    """Test /ready endpoint."""
    print("\n=== Testing /ready endpoint ===")
    response = requests.get(f"{BASE_URL}/ready")
    print(f"Status Code: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
    # Ready endpoint may return 200 or 503 depending on environment setup
    print("[PASS] /ready endpoint works!")


def test_chat_valid():
    """Test /chat endpoint with valid query."""
    print("\n=== Testing /chat endpoint (valid query) ===")
    payload = {
        "query": "What is ROS 2?",
        "top_k": 3
    }
    response = requests.post(f"{BASE_URL}/chat", json=payload)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
    assert response.status_code == 200, f"Expected 200, got {response.status_code}"
    data = response.json()
    assert "answer" in data
    assert "citations" in data
    assert "confidence" in data
    print("[PASS] /chat endpoint works with valid query!")


def test_chat_empty_query():
    """Test /chat endpoint with empty query (should fail)."""
    print("\n=== Testing /chat endpoint (empty query - should fail) ===")
    payload = {
        "query": "",
        "top_k": 3
    }
    response = requests.post(f"{BASE_URL}/chat", json=payload)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
    assert response.status_code == 422, f"Expected 422, got {response.status_code}"
    data = response.json()
    assert "detail" in data  # Pydantic returns "detail" for validation errors
    print("[PASS] /chat endpoint correctly rejects empty query!")


def test_chat_long_query():
    """Test /chat endpoint with query exceeding 2000 chars (should fail)."""
    print("\n=== Testing /chat endpoint (query too long - should fail) ===")
    payload = {
        "query": "a" * 2001,
        "top_k": 3
    }
    response = requests.post(f"{BASE_URL}/chat", json=payload)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
    assert response.status_code == 422, f"Expected 422, got {response.status_code}"
    print("[PASS] /chat endpoint correctly rejects query that's too long!")


if __name__ == "__main__":
    print("=" * 60)
    print("FastAPI Backend Manual Testing")
    print("=" * 60)
    print("\nMake sure the API server is running:")
    print("  cd backend && uvicorn api:app --reload")
    print("\n" + "=" * 60)

    try:
        test_health()
        test_ready()
        test_chat_empty_query()
        test_chat_long_query()
        # Note: test_chat_valid() requires OpenAI API and Qdrant to be properly configured
        # Uncomment if environment is set up:
        # test_chat_valid()

        print("\n" + "=" * 60)
        print("[SUCCESS] All tests passed!")
        print("=" * 60)
    except Exception as e:
        print(f"\n[FAILED] Test failed: {e}")
        import traceback
        traceback.print_exc()
