# Quickstart: FastAPI Backend & Frontend Integration

**Feature**: 007-fastapi-backend-integration
**Date**: 2025-12-28
**Phase**: 1 (Design & Contracts)

## Development Setup

### Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed (for frontend)
- Git repository cloned
- Backend dependencies from Spec 3 (agent.py) already working
- Qdrant Cloud instance configured (from Spec 2)

### Backend Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend/
   ```

2. **Install FastAPI dependencies**:
   ```bash
   # Using uv (recommended, already in project)
   uv add fastapi uvicorn python-multipart

   # Or using pip
   pip install fastapi uvicorn[standard] python-multipart
   ```

3. **Configure environment variables**:
   Create or update `backend/.env`:
   ```bash
   # Existing from Spec 3
   OPENAI_API_KEY=sk-...
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-key

   # New for this feature
   CORS_ORIGINS=http://localhost:3000,http://localhost:3001
   API_HOST=0.0.0.0
   API_PORT=8000
   LOG_LEVEL=INFO
   ```

4. **Run the FastAPI server**:
   ```bash
   # Development mode with auto-reload
   uvicorn api:app --reload --host 0.0.0.0 --port 8000

   # Or using the cli script (if created)
   python api.py
   ```

5. **Verify backend is running**:
   ```bash
   # Health check
   curl http://localhost:8000/health

   # Readiness check
   curl http://localhost:8000/ready

   # Test chat endpoint
   curl -X POST http://localhost:8000/chat \
     -H "Content-Type: application/json" \
     -d '{"query": "What is ROS 2?"}'
   ```

6. **View API documentation**:
   - Swagger UI: http://localhost:8000/docs
   - ReDoc: http://localhost:8000/redoc
   - OpenAPI schema: http://localhost:8000/openapi.json

---

### Frontend Setup

1. **Navigate to frontend directory**:
   ```bash
   cd frontend_book/
   ```

2. **Install dependencies** (if not already done):
   ```bash
   npm install
   ```

3. **Configure API URL**:
   Create or update `frontend_book/.env.local`:
   ```bash
   REACT_APP_API_URL=http://localhost:8000
   ```

4. **Start Docusaurus dev server**:
   ```bash
   npm start
   ```

5. **Verify frontend is running**:
   - Open browser: http://localhost:3000
   - Look for ChatWidget component on pages
   - Test submitting a query through the UI

---

## Testing the Integration

### Manual End-to-End Test

1. **Start both servers**:
   ```bash
   # Terminal 1: Backend
   cd backend && uvicorn api:app --reload

   # Terminal 2: Frontend
   cd frontend_book && npm start
   ```

2. **Open browser** to http://localhost:3000

3. **Submit test queries**:
   - **Book-wide query**: "What is ROS 2?"
   - **Module-filtered query**: Select module "ros2" → "Explain publisher/subscriber"
   - **Selected text query**: Highlight code example → "Explain this code"
   - **Error case**: Submit empty query → should show validation error

4. **Verify expected behavior**:
   - ✅ Responses appear within 5 seconds
   - ✅ Answers include citations with clickable links
   - ✅ Confidence level displayed (high/low/none)
   - ✅ Timing metrics shown in dev console
   - ✅ Error messages are user-friendly (no stack traces)

---

### Automated Testing

1. **Run backend unit tests**:
   ```bash
   cd backend/
   pytest tests/test_api.py -v
   ```

2. **Run backend integration tests**:
   ```bash
   pytest tests/test_integration.py -v
   ```

3. **Run contract tests** (validates OpenAPI schema):
   ```bash
   pytest tests/test_contract.py -v
   ```

4. **Run frontend tests** (if implemented):
   ```bash
   cd frontend_book/
   npm test
   ```

---

## Common Issues

### Issue: CORS Error in Browser

**Symptom**: Browser console shows `CORS policy: No 'Access-Control-Allow-Origin' header`

**Solution**:
- Verify `CORS_ORIGINS` in `backend/.env` includes `http://localhost:3000`
- Restart FastAPI server after changing .env
- Check browser dev tools → Network tab → response headers should include `Access-Control-Allow-Origin`

---

### Issue: 503 Service Unavailable

**Symptom**: All `/chat` requests return 503 status code

**Solution**:
- Check `/ready` endpoint: `curl http://localhost:8000/ready`
- Verify Qdrant connection: check `QDRANT_URL` and `QDRANT_API_KEY` in .env
- Verify OpenAI API key: check `OPENAI_API_KEY` is valid
- Check backend logs for connection errors

---

### Issue: Slow Response Times (>5 seconds)

**Symptom**: Queries take longer than expected

**Solution**:
- Check OpenAI API status: https://status.openai.com
- Verify Qdrant Cloud instance isn't throttled (free tier limits)
- Reduce `top_k` parameter (default 5) if large result sets
- Check network latency to Qdrant and OpenAI

---

### Issue: Empty Responses or No Citations

**Symptom**: Answer is "I couldn't find relevant information" for valid questions

**Solution**:
- Verify Qdrant collection is populated (run `backend/main.py` from Spec 2)
- Check book content is indexed: query Qdrant directly to verify vectors exist
- Test with known good queries: "What is ROS 2?" should always return results
- Verify `min_relevance_score` in agent config isn't too high

---

## Development Workflow

### Adding New Endpoints

1. Update `backend/api.py` with new route handler
2. Update `specs/007-fastapi-backend-integration/contracts/openapi.yaml`
3. Run contract tests to verify schema matches implementation
4. Update frontend `api.ts` to add new client function
5. Test end-to-end flow

### Modifying Request/Response Models

1. Update Pydantic models in `backend/api.py`
2. Run backend tests to verify validation logic
3. Update OpenAPI contract (`openapi.yaml`)
4. Update TypeScript interfaces in frontend `api.ts`
5. Run contract tests to verify compatibility

### Debugging API Issues

1. **Enable debug logging**:
   ```bash
   # In backend/.env
   LOG_LEVEL=DEBUG
   ```

2. **Check FastAPI logs** in terminal where `uvicorn` is running

3. **Use Swagger UI** for interactive testing: http://localhost:8000/docs

4. **Inspect requests in browser dev tools**:
   - Network tab → select `/chat` request
   - Check request payload, response body, status code
   - Look for console errors

---

## Deployment

### Backend Deployment

**Option 1: Vercel (recommended for Python)**
```bash
cd backend/
vercel --prod
```

**Option 2: Render**
- Connect GitHub repo to Render
- Set build command: `pip install -r requirements.txt`
- Set start command: `uvicorn api:app --host 0.0.0.0 --port $PORT`
- Add environment variables from `.env.example`

**Option 3: Railway**
```bash
cd backend/
railway up
```

### Frontend Deployment

**Option 1: Vercel (recommended for Docusaurus)**
```bash
cd frontend_book/
vercel --prod
```

**Option 2: GitHub Pages (static site)**
```bash
npm run build
# Deploy build/ directory to GitHub Pages
```

### Environment Variables for Production

Update environment variables in deployment platform:

**Backend**:
- `OPENAI_API_KEY`: Production OpenAI key
- `QDRANT_URL`: Qdrant Cloud production instance
- `QDRANT_API_KEY`: Qdrant production API key
- `CORS_ORIGINS`: Production frontend domain (e.g., `https://yourdomain.com`)
- `API_HOST`: `0.0.0.0`
- `API_PORT`: Platform-provided port (usually `$PORT` env var)
- `LOG_LEVEL`: `INFO` or `WARNING`

**Frontend**:
- `REACT_APP_API_URL`: Production backend URL (e.g., `https://api.yourdomain.com`)

---

## Next Steps

After completing quickstart setup:

1. Run `/sp.tasks` to generate implementation tasks
2. Follow TDD workflow:
   - Write test (red)
   - Implement feature (green)
   - Refactor (refactor)
3. Commit changes with references to spec: `feat(007): add FastAPI chat endpoint`
4. Run full test suite before creating PR
5. Deploy to production after PR merge

---

## Resources

- FastAPI documentation: https://fastapi.tiangolo.com
- Pydantic v2 docs: https://docs.pydantic.dev/latest/
- Uvicorn docs: https://www.uvicorn.org
- OpenAPI 3.1 spec: https://spec.openapis.org/oas/v3.1.0
- Docusaurus docs: https://docusaurus.io
- React TypeScript: https://react-typescript-cheatsheet.netlify.app

---

**Last Updated**: 2025-12-28
**Status**: Ready for implementation phase (/sp.tasks)
