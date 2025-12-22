# Environment Variables Migration Guide

## Overview
All hardcoded URLs have been replaced with environment variables for better configuration management and deployment flexibility.

## Files Updated

### Frontend Files (React/Docusaurus)
1. **src/auth/context/AuthContext.js**
   - Added `AUTH_BACKEND_URL` constant
   - Replaced 5 hardcoded URLs with environment variable
   - Uses `NEXT_PUBLIC_AUTH_BACKEND_URL`

2. **src/auth/client.ts**
   - Replaced hardcoded baseURL with environment variable
   - Uses `NEXT_PUBLIC_AUTH_BACKEND_URL`

3. **src/auth/api/api-client.ts**
   - Replaced `REACT_APP_API_URL` with `NEXT_PUBLIC_AUTH_BACKEND_URL`

4. **src/theme/Root.tsx**
   - Added `RAG_BACKEND_URL` constant
   - Replaced chat endpoint URLs with environment variable
   - Uses `NEXT_PUBLIC_RAG_BACKEND_URL`

5. **src/personalization/services/personalization.service.ts**
   - Replaced hardcoded personalization endpoint
   - Uses `NEXT_PUBLIC_RAG_BACKEND_URL`

6. **src/personalization/components/PersonalizeChapterButton.js**
   - Replaced hardcoded personalization endpoint
   - Uses `NEXT_PUBLIC_RAG_BACKEND_URL`

7. **frontend/widget.js**
   - Replaced 3 instances of hardcoded URLs
   - Uses `NEXT_PUBLIC_RAG_BACKEND_URL` or falls back to data attribute

### Backend Files
1. **api-server.mjs (Better Auth backend)**
   - Console log URLs now use `BETTER_AUTH_URL` environment variable
   - Already uses `FRONTEND_URL` for CORS

2. **fastapi_app/app.py (RAG backend)**
   - Removed ALL authentication logic
   - Replaced hardcoded CORS origins with `ALLOWED_ORIGINS` environment variable
   - Added `LLM_BASE_URL` environment variable
   - Uses `PORT` and `HOST` environment variables

## Environment Variables Reference

### Frontend Variables (prefix: NEXT_PUBLIC_)
```bash
# Authentication backend (Better Auth)
NEXT_PUBLIC_AUTH_BACKEND_URL=http://localhost:8000

# RAG backend (FastAPI)
NEXT_PUBLIC_RAG_BACKEND_URL=http://localhost:8000

# Frontend URL
NEXT_PUBLIC_FRONTEND_URL=http://localhost:3002
```

### Better Auth Backend Variables
```bash
DATABASE_URL=postgresql://...
BETTER_AUTH_SECRET=...
BETTER_AUTH_URL=http://localhost:8000
FRONTEND_URL=http://localhost:3002
NODE_ENV=development
```

### FastAPI Backend Variables
```bash
PORT=8000
HOST=0.0.0.0
ENVIRONMENT=development
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,http://localhost:3002
GEMINI_API_KEY=...
LLM_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GROQ_API_KEY=...
QDRANT_URL=...
QDRANT_HOST=...
QDRANT_PORT=6333
QDRANT_API_KEY=...
```

## Setup Instructions

1. **Copy .env.example to .env**
   ```bash
   cp .env.example .env
   ```

2. **Update .env with your actual values**
   - Replace placeholder URLs with your deployment URLs
   - Add your API keys
   - Configure database connection strings

3. **For Local Development**
   ```bash
   NEXT_PUBLIC_AUTH_BACKEND_URL=http://localhost:8000
   NEXT_PUBLIC_RAG_BACKEND_URL=http://localhost:8000
   ```

4. **For Production Deployment**
   ```bash
   NEXT_PUBLIC_AUTH_BACKEND_URL=https://your-auth-backend.vercel.app
   NEXT_PUBLIC_RAG_BACKEND_URL=https://your-rag-backend.vercel.app
   BETTER_AUTH_URL=https://your-auth-backend.vercel.app
   FRONTEND_URL=https://your-frontend.vercel.app
   ALLOWED_ORIGINS=https://your-frontend.vercel.app
   ```

## Deployment Configuration

### Vercel Deployment
When deploying to Vercel, set these environment variables in the Vercel dashboard:

**For Better Auth Backend:**
- `DATABASE_URL`
- `BETTER_AUTH_SECRET`
- `BETTER_AUTH_URL` (your Vercel backend URL)
- `FRONTEND_URL` (your Vercel frontend URL)
- `NODE_ENV=production`

**For FastAPI Backend:**
- `ALLOWED_ORIGINS` (your frontend URL)
- `GEMINI_API_KEY`
- `GROQ_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `ENVIRONMENT=production`

**For Frontend (Docusaurus):**
- `NEXT_PUBLIC_AUTH_BACKEND_URL` (your auth backend URL)
- `NEXT_PUBLIC_RAG_BACKEND_URL` (your RAG backend URL)

## Changes Summary

### What Was Removed
- ❌ All authentication logic from FastAPI (register, login, me endpoints)
- ❌ All hardcoded URLs (`https://fastapi-backend-for-book.vercel.app`, etc.)
- ❌ Mock user database and token validation in FastAPI

### What Was Added
- ✅ Environment variable support for ALL URLs
- ✅ Fallback values for local development
- ✅ Comprehensive .env.example file
- ✅ Clean separation: FastAPI = RAG only, Node.js = Auth only

## Testing

After updating your .env file, test that:
1. Frontend can connect to both backends
2. Authentication flows work (login/signup)
3. RAG chat functionality works
4. Personalization endpoints work
5. Session persistence works across page reloads

## Notes
- The `NEXT_PUBLIC_` prefix is required for environment variables to be accessible in the browser
- Backend environment variables (without prefix) are server-side only
- Always use environment variables - never hardcode URLs in code
