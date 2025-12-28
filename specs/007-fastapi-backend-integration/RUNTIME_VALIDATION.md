# Runtime Validation Checklist

This document outlines the remaining validation tasks that require a running server with live dependencies (OpenAI API, Qdrant Cloud).

## Prerequisites

Before running these validations:

1. ✅ Backend implementation complete (backend/api.py)
2. ✅ Frontend implementation complete (ChatWidget component)
3. ✅ Test files created (test_api.py, test_integration.py, test_contract.py)
4. ✅ Environment configured (.env with valid credentials)
5. ⏳ OpenAI API key with available credits
6. ⏳ Qdrant Cloud instance populated with book content
7. ⏳ Both backend and frontend servers running

## Task Checklist

### T058: Run Full Test Suite ⏳

**Objective**: Verify 100% test pass rate with live dependencies

**Steps**:
1. Start backend server:
   ```bash
   cd backend/
   uvicorn api:app --reload --host 0.0.0.0 --port 8000
   ```

2. Run pytest with coverage:
   ```bash
   cd backend/
   pytest tests/ -v --cov=. --cov-report=html
   ```

3. Verify results:
   - All tests pass (0 failures)
   - Coverage report generated in htmlcov/
   - No warnings or errors in output

**Success Criteria**:
- ✅ All unit tests pass (test_api.py)
- ✅ All integration tests pass (test_integration.py)
- ✅ All contract tests pass (test_contract.py)
- ✅ Code coverage > 80%

**Notes**:
- Some tests are currently marked as TODO and need uncommenting
- Ensure .env has valid OPENAI_API_KEY and QDRANT credentials
- Mock tests can run without live API, but integration tests require it

---

### T059: Test /health Endpoint Response Time ⏳

**Objective**: Verify /health endpoint responds within 500ms (SC-008)

**Steps**:
1. Start backend server (if not running)

2. Test response time with curl:
   ```bash
   curl -w "\nTime: %{time_total}s\n" http://localhost:8000/health
   ```

3. Or use Python script:
   ```python
   import requests
   import time

   times = []
   for _ in range(10):
       start = time.time()
       response = requests.get("http://localhost:8000/health")
       elapsed = (time.time() - start) * 1000
       times.append(elapsed)
       print(f"Response time: {elapsed:.2f}ms")

   avg_time = sum(times) / len(times)
   print(f"Average: {avg_time:.2f}ms")
   print(f"Max: {max(times):.2f}ms")
   ```

**Success Criteria**:
- ✅ Average response time < 500ms
- ✅ P95 response time < 500ms
- ✅ All requests return 200 status code
- ✅ Response body contains {"status": "ok"}

**Expected Result**: ~10-50ms response time (very fast, no dependencies)

---

### T060: Test /ready Endpoint Response Time ⏳

**Objective**: Verify /ready endpoint responds within 500ms (SC-008)

**Steps**:
1. Start backend server (if not running)

2. Test response time:
   ```bash
   curl -w "\nTime: %{time_total}s\n" http://localhost:8000/ready
   ```

3. Or use Python script:
   ```python
   import requests
   import time

   times = []
   for _ in range(10):
       start = time.time()
       response = requests.get("http://localhost:8000/ready")
       elapsed = (time.time() - start) * 1000
       times.append(elapsed)
       status = "ready" if response.status_code == 200 else "not ready"
       print(f"Response time: {elapsed:.2f}ms, Status: {status}")

   avg_time = sum(times) / len(times)
   print(f"Average: {avg_time:.2f}ms")
   ```

**Success Criteria**:
- ✅ Average response time < 500ms
- ✅ P95 response time < 500ms
- ✅ Response includes all health checks (agent, retrieval, env)
- ✅ Returns 200 when all checks pass, 503 when degraded

**Expected Result**: ~100-300ms response time (checks Qdrant connectivity)

---

### T063: Manual End-to-End Test ⏳

**Objective**: Follow quickstart.md validation steps

**Steps**:
1. Start backend:
   ```bash
   cd backend/
   uvicorn api:app --reload
   ```

2. Start frontend (separate terminal):
   ```bash
   cd frontend_book/
   npm start
   ```

3. Test backend endpoints:
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

4. Test frontend:
   - Open http://localhost:3000 in browser
   - Navigate to a page with ChatWidget
   - Submit test query: "What is ROS 2?"
   - Verify response appears with citations
   - Test error handling: submit empty query
   - Test selected text: highlight text, ask about it

5. Test API documentation:
   - Open http://localhost:8000/docs (Swagger UI)
   - Try /chat endpoint with example request
   - Verify OpenAPI schema loads correctly

**Success Criteria**:
- ✅ Backend starts without errors
- ✅ Frontend starts without errors
- ✅ Health checks return 200
- ✅ Chat query returns answer with citations
- ✅ Error queries return user-friendly messages
- ✅ Frontend ChatWidget displays correctly
- ✅ No CORS errors in browser console
- ✅ API documentation accessible

---

### T064: Load Testing (50 Concurrent Requests) ⏳

**Objective**: Verify system handles 50 concurrent requests (SC-002)

**Steps**:
1. Install load testing tool:
   ```bash
   pip install locust
   ```

2. Create locustfile.py:
   ```python
   from locust import HttpUser, task, between

   class ChatUser(HttpUser):
       wait_time = between(1, 3)

       @task
       def chat_query(self):
           self.client.post("/chat", json={
               "query": "What is ROS 2?",
               "top_k": 5
           })

       @task(2)
       def health_check(self):
           self.client.get("/health")
   ```

3. Run load test:
   ```bash
   locust -f locustfile.py --host=http://localhost:8000 --users 50 --spawn-rate 10 --run-time 60s
   ```

4. Monitor results in Locust web UI (http://localhost:8089)

**Success Criteria**:
- ✅ 0% failure rate for 50 concurrent users
- ✅ Average response time remains acceptable
- ✅ No 503 Service Unavailable errors
- ✅ Backend logs show no errors
- ✅ System remains stable throughout test

**Expected Metrics**:
- Request rate: ~10-20 req/sec
- Average response time: 2-5 seconds (includes OpenAI API latency)
- Failure rate: 0%

---

### T065: Verify 95% Requests < 5 Seconds ⏳

**Objective**: Verify SC-001 (95% of requests complete within 5 seconds under normal load)

**Steps**:
1. Run extended load test (10 minutes):
   ```bash
   locust -f locustfile.py --host=http://localhost:8000 \
          --users 10 --spawn-rate 2 --run-time 600s --headless \
          --csv=results
   ```

2. Analyze results:
   ```python
   import pandas as pd

   # Load Locust results
   df = pd.read_csv('results_stats.csv')

   # Filter /chat requests
   chat_requests = df[df['Name'] == '/chat']

   # Check percentiles
   p95 = chat_requests['95%'].values[0]
   p99 = chat_requests['99%'].values[0]

   print(f"P95 response time: {p95}ms")
   print(f"P99 response time: {p99}ms")
   print(f"SC-001 PASS: {p95 < 5000}")
   ```

3. Alternative: Use curl with timing:
   ```bash
   for i in {1..100}; do
     curl -w "%{time_total}\n" -o /dev/null -s \
          -X POST http://localhost:8000/chat \
          -H "Content-Type: application/json" \
          -d '{"query": "What is ROS 2?", "top_k": 5}'
   done | sort -n | awk 'BEGIN{c=0} {total+=$1; times[c++]=$1} END{
     p95_idx=int(c*0.95);
     print "P95:", times[p95_idx], "seconds";
     print "PASS:", (times[p95_idx] < 5 ? "YES" : "NO")
   }'
   ```

**Success Criteria**:
- ✅ P95 response time < 5000ms (5 seconds)
- ✅ P99 response time < 8000ms (8 seconds)
- ✅ Average response time < 3000ms (3 seconds)
- ✅ No timeouts or failures

**Notes**:
- Response time depends on OpenAI API latency (typically 1-3 seconds)
- Qdrant retrieval typically adds 100-300ms
- Total time budget: 5 seconds for P95

---

## Validation Summary

After completing all tasks above, mark them as complete in tasks.md:

- [ ] T058 ✓ All tests pass with live dependencies
- [ ] T059 ✓ /health responds < 500ms
- [ ] T060 ✓ /ready responds < 500ms
- [ ] T063 ✓ Manual end-to-end test successful
- [ ] T064 ✓ 50 concurrent users handled successfully
- [ ] T065 ✓ P95 response time < 5 seconds

## Common Issues

### High Response Times

**Cause**: OpenAI API latency or Qdrant Cloud throttling

**Solution**:
- Check OpenAI API status: https://status.openai.com
- Verify Qdrant plan (free tier has limits)
- Reduce top_k parameter
- Enable caching for repeated queries

### Test Failures

**Cause**: Missing credentials or dependencies

**Solution**:
- Verify .env has valid OPENAI_API_KEY
- Verify Qdrant collection is populated
- Check test_client fixture in conftest.py
- Uncomment TODO tests in test files

### CORS Errors

**Cause**: Frontend/backend origin mismatch

**Solution**:
- Update CORS_ORIGINS in backend/.env
- Restart backend server
- Clear browser cache

## Next Steps

After validation:
1. ✅ All runtime tasks marked complete in tasks.md
2. ✅ Load test results documented
3. ✅ Performance metrics recorded
4. ✅ Any issues resolved or documented
5. ✅ Ready for production deployment

## Deployment Checklist

Once all validations pass:
- [ ] Deploy backend to production (Vercel/Render/Railway)
- [ ] Deploy frontend to production (Vercel/GitHub Pages)
- [ ] Configure production environment variables
- [ ] Set up monitoring and alerting
- [ ] Document production URLs
- [ ] Create user documentation
- [ ] Announce feature launch

---

**Last Updated**: 2025-12-28
**Status**: Ready for runtime validation
