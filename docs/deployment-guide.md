# RAG Chatbot Deployment Guide

This guide provides comprehensive instructions for deploying the RAG Chatbot system to production environments using Render for the backend and Vercel for the frontend.

## Table of Contents
- [Architecture Overview](#architecture-overview)
- [Prerequisites](#prerequisites)
- [Backend Deployment (Render)](#backend-deployment-render)
- [Frontend Deployment (Vercel)](#frontend-deployment-vercel)
- [Environment Variables](#environment-variables)
- [Configuration Settings](#configuration-settings)
- [Monitoring and Maintenance](#monitoring-and-maintenance)
- [Troubleshooting](#troubleshooting)

## Architecture Overview

The RAG Chatbot system consists of:

1. **Backend API**: FastAPI application hosted on Render
   - Handles RAG operations (embedding, search, ask-agent)
   - Integrates with Qdrant vector database
   - Uses Redis for caching and session management
   - Google Gemini for LLM operations

2. **Frontend Widget**: React-based chat widget hosted on Vercel
   - Embedded in Docusaurus documentation site
   - Communicates with backend via REST API
   - Supports text selection and context-aware responses

## Prerequisites

Before deployment, ensure you have:

- Google Gemini API key
- Qdrant Cloud account (or self-hosted instance)
- Redis instance (for caching and sessions)
- GitHub account for deployment
- Render account (for backend)
- Vercel account (for frontend)

## Backend Deployment (Render)

### Using Render Blueprint (Recommended)

1. Create a `render.yaml` file in your backend directory:

```yaml
services:
  - type: web
    name: rag-chatbot-api
    runtime: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn src.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: PYTHON_VERSION
        value: 3.11
      - key: GEMINI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: REDIS_HOST
        value: localhost
      - key: REDIS_PORT
        value: 6379
      - key: REDIS_PASSWORD
        sync: false
      - key: DEBUG
        value: False
    healthCheckPath: /api/v1/health
    env: python
    plan: free
    autoDeploy: false
    region: oregon
```

2. Connect your GitHub repository to Render
3. Render will automatically detect the blueprint and deploy the service
4. Add the required environment variables in the Render dashboard

### Using Docker (Alternative)

1. Ensure your Dockerfile is properly configured for Render:

```Dockerfile
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV PORT=10000

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy project
COPY . .

# Create non-root user
RUN adduser --disabled-password --gecos '' appuser
RUN chown -R appuser:appuser /app
USER appuser

# Expose port
EXPOSE $PORT

# Run the application
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "$PORT"]
```

2. Push your code to GitHub
3. Connect to Render and deploy

## Frontend Deployment (Vercel)

### Using Vercel CLI

1. Install Vercel CLI:
```bash
npm i -g vercel
```

2. Navigate to the frontend directory:
```bash
cd frontend
```

3. Deploy to Vercel:
```bash
vercel --prod
```

### Using Vercel Dashboard

1. Create a `vercel.json` file in your frontend directory:

```json
{
  "version": 2,
  "name": "rag-chatbot-frontend",
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distPath": "dist"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ],
  "env": {
    "NODE_VERSION": "18.x"
  }
}
```

2. Push your code to GitHub
3. Import the project in Vercel dashboard
4. Vercel will automatically build and deploy the frontend

## Environment Variables

### Backend (Render)

| Variable | Required | Description |
|----------|----------|-------------|
| GEMINI_API_KEY | Yes | Google Gemini API key |
| QDRANT_URL | Yes | Qdrant database URL |
| QDRANT_API_KEY | No | Qdrant API key (if using cloud) |
| REDIS_HOST | No | Redis host (default: localhost) |
| REDIS_PORT | No | Redis port (default: 6379) |
| REDIS_PASSWORD | No | Redis password |
| REDIS_DB | No | Redis database number (default: 0) |
| CACHE_TTL | No | Cache time-to-live in seconds (default: 3600) |
| DEBUG | No | Enable debug mode (default: False) |

### Frontend (Vercel)

| Variable | Required | Description |
|----------|----------|-------------|
| NEXT_PUBLIC_BACKEND_URL | Yes | Backend API URL |
| NEXT_PUBLIC_CHATBOT_ENABLED | No | Enable chatbot widget (default: true) |

## Configuration Settings

### Backend Configuration

The backend uses Pydantic Settings for configuration. You can override settings via environment variables:

```python
class Settings(BaseSettings):
    # Qdrant settings
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: Optional[str] = None

    # Gemini settings
    gemini_api_key: str
    gemini_model: str = "gemini-pro"

    # Embedding model settings
    embedding_model: str = "gemini-embedding-001"

    # Redis settings for caching
    redis_host: str = "localhost"
    redis_port: int = 6379
    redis_db: int = 0
    redis_password: Optional[str] = None
    cache_ttl: int = 3600  # Cache time-to-live in seconds (1 hour)

    # Application settings
    app_name: str = "RAG Chatbot API"
    debug: bool = False
    max_concurrent_users: int = 50
```

### Frontend Configuration

The frontend chat widget can be configured via data attributes or JavaScript initialization:

```html
<!-- Method 1: Data attributes -->
<div data-chatbot-widget
     data-backend-url="https://your-backend-url.com"
     data-theme="auto">
</div>

<!-- Method 2: JavaScript initialization -->
<script>
  window.ChatbotWidget.init({
    backendUrl: 'https://your-backend-url.com',
    theme: 'auto',
    position: 'bottom-right'
  });
</script>
```

## Monitoring and Maintenance

### Backend Metrics

The backend exposes metrics endpoints:

- `/api/v1/metrics/summary` - Overall application metrics
- `/api/v1/metrics/endpoint?endpoint=/api/v1/ask-agent` - Specific endpoint metrics

### Health Checks

The application provides a health check endpoint:
- `GET /api/v1/health` - Returns health status

### Performance Monitoring

- Response times are logged for requests > 2 seconds
- Error rates are tracked and reported
- Request counts per minute are monitored

## Troubleshooting

### Common Issues

#### Backend Deployment Fails
- Check that all required environment variables are set
- Verify Python version compatibility (3.11 recommended)
- Ensure requirements.txt is properly formatted

#### Frontend Not Connecting to Backend
- Verify `NEXT_PUBLIC_BACKEND_URL` is correctly set
- Check CORS configuration in backend
- Ensure backend is accessible from frontend domain

#### Slow Response Times
- Verify Qdrant connection and performance
- Check Redis connection for caching
- Review embedding model performance

#### Chat Widget Not Appearing
- Verify the widget script is properly loaded
- Check that the integration code is placed correctly in HTML
- Ensure no JavaScript errors prevent initialization

### Debugging Tips

1. Check Render logs for backend issues:
   ```bash
   render service logs --serviceId <service-id>
   ```

2. Use browser developer tools for frontend issues

3. Test API endpoints directly using tools like curl or Postman

4. Monitor the metrics endpoints for performance issues

### Support Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Render Documentation](https://render.com/docs)
- [Vercel Documentation](https://vercel.com/docs)
- [Qdrant Documentation](https://qdrant.tech/documentation/)