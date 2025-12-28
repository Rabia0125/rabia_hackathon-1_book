# Deployment Guide: RAG Chatbot

## Backend Deployment (FastAPI)

### Option 1: Deploy to Render (Recommended - Free Tier)

1. **Push your code to GitHub** (if not already done)
   ```bash
   git add .
   git commit -m "Add deployment configuration"
   git push origin main
   ```

2. **Create Render Account**
   - Go to https://render.com
   - Sign up with your GitHub account

3. **Create New Web Service**
   - Click "New +" → "Web Service"
   - Connect your GitHub repository: `My-Book`
   - Configure:
     - **Name**: `rag-chatbot-backend`
     - **Region**: Oregon (US West)
     - **Branch**: `main`
     - **Root Directory**: Leave empty
     - **Runtime**: Python 3
     - **Build Command**: `pip install -r backend/requirements.txt`
     - **Start Command**: `cd backend && uvicorn api:app --host 0.0.0.0 --port $PORT`

4. **Add Environment Variables** (in Render dashboard)
   - `OPENAI_API_KEY` = (from your .env file)
   - `QDRANT_URL` = (from your .env file)
   - `QDRANT_API_KEY` = (from your .env file)
   - `COHERE_API_KEY` = (from your .env file)
   - `COLLECTION_NAME` = `physical-ai-book`
   - `CORS_ORIGINS` = `https://rabia-hackathon-1-book.vercel.app,http://localhost:3000`
   - `LOG_LEVEL` = `INFO`
   - `API_HOST` = `0.0.0.0`

5. **Deploy**
   - Click "Create Web Service"
   - Wait for deployment (5-10 minutes)
   - Copy your backend URL (e.g., `https://rag-chatbot-backend.onrender.com`)

### Option 2: Deploy to Railway

1. **Create Railway Account**
   - Go to https://railway.app
   - Sign up with GitHub

2. **New Project**
   - Click "New Project" → "Deploy from GitHub repo"
   - Select your `My-Book` repository

3. **Configure Service**
   - Railway will auto-detect Python
   - Add the same environment variables as above
   - Railway will use the `Procfile` and `railway.json` automatically

4. **Deploy**
   - Railway will deploy automatically
   - Copy your backend URL from the dashboard

---

## Frontend Configuration

### Step 1: Update Environment Variable

Update `frontend_book/.env.local`:
```bash
# Replace with your actual backend URL from Render/Railway
REACT_APP_API_URL=https://rag-chatbot-backend.onrender.com
```

### Step 2: Test Locally (Optional)
```bash
cd frontend_book
npm run start
# Visit http://localhost:3000 and test the chatbot
```

### Step 3: Deploy to Vercel

```bash
cd frontend_book
npm run build

# Deploy to Vercel
# Option A: Using Vercel CLI
vercel --prod

# Option B: Push to GitHub (auto-deploy)
git add .
git commit -m "Configure production API URL"
git push origin main
```

### Step 4: Add Environment Variable in Vercel Dashboard

1. Go to https://vercel.com/dashboard
2. Select your project: `rabia-hackathon-1-book`
3. Go to "Settings" → "Environment Variables"
4. Add:
   - **Key**: `REACT_APP_API_URL`
   - **Value**: `https://rag-chatbot-backend.onrender.com` (your backend URL)
   - **Environment**: Production, Preview, Development
5. Click "Save"
6. Redeploy: Go to "Deployments" → Click "..." → "Redeploy"

---

## Verification

1. **Check Backend Health**
   ```bash
   curl https://rag-chatbot-backend.onrender.com/health
   # Should return: {"status":"ok"}
   ```

2. **Check Backend Readiness**
   ```bash
   curl https://rag-chatbot-backend.onrender.com/ready
   # Should return ready status with all checks passing
   ```

3. **Test Chat Endpoint**
   ```bash
   curl -X POST https://rag-chatbot-backend.onrender.com/chat \
     -H "Content-Type: application/json" \
     -d '{"query": "What is ROS2?", "top_k": 3}'
   ```

4. **Visit Your Website**
   - Go to: https://rabia-hackathon-1-book.vercel.app
   - Look for the chatbot widget in the bottom-right corner
   - Try asking: "What is ROS2?" or "Explain humanoid robots"

---

## Troubleshooting

### Chatbot Still Not Visible

1. **Check Browser Console** (F12 → Console tab)
   - Look for CORS errors
   - Look for network errors to the API

2. **Check Network Tab** (F12 → Network tab)
   - Make requests to `/chat` endpoint
   - Verify the request URL matches your backend

3. **Verify CORS in Backend**
   - Ensure `CORS_ORIGINS` includes your Vercel domain
   - No trailing slashes in URLs

### Backend Errors

1. **Check Render Logs**
   - Go to Render dashboard → Your service → Logs
   - Look for startup errors or runtime errors

2. **Verify Environment Variables**
   - All API keys are set correctly
   - No quotes around values in Render/Railway dashboard

### API Keys Issues

- If you see "AGENT_ERROR", check `OPENAI_API_KEY` is valid
- If you see "RETRIEVAL_ERROR", check Qdrant credentials
