# FastAPI Backend for RAG Chatbot

FastAPI backend service that exposes REST API endpoints for the Physical AI & Robotics book RAG chatbot. Connects the Docusaurus frontend to the RAG agent with retrieval-augmented generation capabilities.

## Features

- ✅ REST API with `/chat`, `/health`, and `/ready` endpoints
- ✅ Natural language query processing with RAG
- ✅ Citation-backed answers from book content
- ✅ Support for book-wide and module-filtered queries
- ✅ Graceful error handling with user-friendly messages
- ✅ CORS enabled for frontend integration
- ✅ JSON logging for observability
- ✅ Health monitoring for deployment

## Prerequisites

- Python 3.11 or higher
- OpenAI API key
- Qdrant Cloud instance (from Spec 2 setup)
- Existing `agent.py` and `retrieve.py` from previous specs

## Quick Start

### 1. Install Dependencies

```bash
cd backend/
pip install -r requirements.txt
# Or using uv (recommended)
uv sync
```

### 2. Configure Environment

Create a `.env` file or copy from `.env.example`:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# Required
OPENAI_API_KEY=sk-your-openai-api-key-here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=http://localhost:3000
LOG_LEVEL=INFO
```

### 3. Start the API Server

```bash
# Development mode with auto-reload
uvicorn api:app --reload --host 0.0.0.0 --port 8000

# Or using Python directly
python api.py
```

The API will be available at http://localhost:8000

### 4. Test the API

**Using curl**:
```bash
# Health check
curl http://localhost:8000/health

# Readiness check
curl http://localhost:8000/ready

# Chat query
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "top_k": 5}'
```

**Using the interactive docs**:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

**Using the manual test script**:
```bash
python test_api_manual.py
```

## API Endpoints

### POST /chat

Submit a natural language query about the book content.

**Request**:
```json
{
  "query": "What is ROS 2?",
  "selected_text": "optional highlighted text for context",
  "module_filter": "ros2",
  "top_k": 5
}
```

**Response**:
```json
{
  "answer": "ROS 2 is the second generation of the Robot Operating System...",
  "citations": [
    {
      "page_title": "Introduction to ROS 2",
      "page_url": "/docs/ros2/intro",
      "module_name": "ros2"
    }
  ],
  "confidence": "high",
  "retrieval_time_ms": 145.3,
  "generation_time_ms": 820.5,
  "total_time_ms": 965.8
}
```

### GET /health

Liveness probe for container orchestrators. Returns 200 if the process is alive.

**Response**:
```json
{
  "status": "ok"
}
```

### GET /ready

Readiness probe for load balancers. Verifies all dependencies are healthy.

**Response**:
```json
{
  "status": "ready",
  "checks": {
    "agent": true,
    "retrieval": true,
    "env": true
  },
  "version": "1.0.0"
}
```

## Configuration

### Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `OPENAI_API_KEY` | Yes | - | OpenAI API key for agent |
| `QDRANT_URL` | Yes | - | Qdrant Cloud instance URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |
| `API_HOST` | No | `0.0.0.0` | API server bind address |
| `API_PORT` | No | `8000` | API server port |
| `CORS_ORIGINS` | No | `http://localhost:3000` | Comma-separated allowed origins |
| `LOG_LEVEL` | No | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR) |

### CORS Configuration

For production deployment, update `CORS_ORIGINS` to include your frontend domain:

```env
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com
```

## Development

### Running Tests

```bash
# Run all tests
pytest backend/tests/ -v

# Run specific test file
pytest backend/tests/test_api.py -v

# Run with coverage
pytest backend/tests/ --cov=backend --cov-report=html
```

### Code Quality

```bash
# Format code
black backend/

# Lint code
ruff check backend/

# Type checking
mypy backend/
```

## Deployment

### Option 1: Vercel

```bash
cd backend/
vercel --prod
```

### Option 2: Render

1. Connect GitHub repo to Render
2. Set build command: `pip install -r requirements.txt`
3. Set start command: `uvicorn api:app --host 0.0.0.0 --port $PORT`
4. Add environment variables from `.env.example`

### Option 3: Railway

```bash
cd backend/
railway up
```

### Option 4: Docker

```bash
# Build image
docker build -t rag-chatbot-api .

# Run container
docker run -p 8000:8000 --env-file .env rag-chatbot-api
```

## Troubleshooting

### CORS Errors

**Symptom**: Browser console shows `CORS policy: No 'Access-Control-Allow-Origin' header`

**Solution**:
- Verify `CORS_ORIGINS` in `.env` includes your frontend URL
- Restart the API server after changing `.env`
- Check browser dev tools → Network tab → response headers

### 503 Service Unavailable

**Symptom**: All `/chat` requests return 503

**Solution**:
- Check `/ready` endpoint: `curl http://localhost:8000/ready`
- Verify Qdrant connection (check `QDRANT_URL` and `QDRANT_API_KEY`)
- Verify OpenAI API key is valid
- Check API logs for connection errors

### Slow Response Times

**Symptom**: Queries take longer than 5 seconds

**Solution**:
- Check OpenAI API status: https://status.openai.com
- Verify Qdrant Cloud instance isn't throttled (free tier limits)
- Reduce `top_k` parameter if needed
- Check network latency to Qdrant and OpenAI

### Empty Responses

**Symptom**: Answer is "I couldn't find relevant information"

**Solution**:
- Verify Qdrant collection is populated (run Spec 2 pipeline)
- Test with known good queries: "What is ROS 2?" should work
- Check `min_relevance_score` in agent config

## Architecture

```
┌─────────────┐      HTTP      ┌──────────────┐
│  Docusaurus │ ─────────────> │   FastAPI    │
│  Frontend   │ <───────────── │   Backend    │
└─────────────┘     JSON       └──────┬───────┘
                                      │
                                      ├─> agent.py (Spec 3)
                                      │   └─> OpenAI Agents SDK
                                      │
                                      └─> retrieve.py (Spec 2)
                                          └─> Qdrant Cloud
```

## API Specification

The complete OpenAPI schema is available at:
- Interactive docs: http://localhost:8000/docs
- OpenAPI JSON: http://localhost:8000/openapi.json
- OpenAPI YAML: `specs/007-fastapi-backend-integration/contracts/openapi.yaml`

## License

[Your License Here]

## Support

For issues or questions:
- GitHub Issues: [Your Repo URL]
- Documentation: `specs/007-fastapi-backend-integration/`
