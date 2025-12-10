# Quickstart Guide: RAG Chatbot for Digital Book Website

## Overview
This guide provides a quick setup and deployment process for the RAG chatbot system that integrates with your digital book website.

## Prerequisites
- Python 3.11+ installed
- Node.js 16+ installed
- Access to Google AI Studio for Gemini API key
- Qdrant Cloud Free Tier account
- Render.com account (for backend deployment)
- Vercel account (for frontend deployment)

## Local Development Setup

### 1. Clone and Prepare the Repository
```bash
git clone <your-repo-url>
cd <repo-directory>
```

### 2. Backend Setup (FastAPI)
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export GEMINI_API_KEY="your-gemini-api-key"
export QDRANT_URL="your-qdrant-cluster-url"
export QDRANT_API_KEY="your-qdrant-api-key"

# Run the backend locally
uvicorn src.main:app --reload --port 8000
```

### 3. Frontend Widget Setup
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Build the widget
npm run build

# The widget can be served locally for testing
npm run serve
```

## API Endpoints

### Backend API (FastAPI)
- `POST /embed`: Generate embeddings for text
- `POST /search`: Search for similar book chunks
- `POST /ask-agent`: Process user query with RAG

### Example API Usage
```bash
# Generate embedding
curl -X POST http://localhost:8000/embed \
  -H "Content-Type: application/json" \
  -d '{"text": "Your question here"}'

# Search for similar content
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{"query_embedding": [0.1, 0.2, 0.3], "top_k": 5}'

# Ask the chatbot
curl -X POST http://localhost:8000/ask-agent \
  -H "Content-Type: application/json" \
  -d '{"question": "What is the main concept?", "selected_text": ""}'
```

## Book Content Indexing

### 1. Prepare Book Content
Ensure your book content is in Markdown format in the `book/docs/` directory.

### 2. Run Chunking and Indexing Script
```bash
cd backend
python scripts/upload_chunks.py
```

This script will:
- Read all Markdown files in the book directory
- Split content into appropriate chunks
- Generate embeddings using the configured model
- Upload chunks to Qdrant vector database

## Frontend Integration

### 1. Add Widget to Docusaurus
Add the following to your `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  scripts: [
    '/js/chatbot-widget.js'
  ],
  // ... rest of config
};
```

### 2. Place Widget Script
Copy the built widget to your Docusaurus static directory:
```bash
cp frontend/dist/widget.js static/js/
```

## Environment Variables

### Backend (.env file in backend directory)
```env
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
EMBEDDING_MODEL=gemini-embedding-001
BACKEND_URL=http://localhost:8000  # or your deployed backend URL
```

### Frontend Configuration
The frontend widget will need to know the backend API URL. This can be configured in the widget initialization:

```javascript
window.ChatbotWidget.init({
  backendUrl: 'http://localhost:8000',  // or your deployed backend URL
  theme: 'auto'  // 'light', 'dark', or 'auto'
});
```

## Deployment

### 1. Deploy Backend to Render
1. Push your code to a Git repository
2. Connect Render to your repository
3. Set environment variables in Render dashboard
4. Configure build command: `pip install -r requirements.txt`
5. Configure start command: `uvicorn src.main:app:app --host 0.0.0.0 --port $PORT`

### 2. Deploy Frontend to Vercel
1. Push your frontend code to a Git repository
2. Connect Vercel to your repository
3. Set build command: `npm run build`
4. Set output directory: `dist`

### 3. Update Configuration
After deployment, update the frontend to point to your deployed backend URL.

## Testing the System

### 1. Verify Backend
```bash
# Test the backend API
curl http://your-backend-url/health
```

### 2. Test RAG Pipeline
```bash
# Test the full RAG pipeline
curl -X POST http://your-backend-url/ask-agent \
  -H "Content-Type: application/json" \
  -d '{"question": "What is this book about?"}'
```

### 3. Test Frontend Widget
Visit your book website and verify the chatbot widget appears and functions correctly.

## Troubleshooting

### Common Issues
1. **API Keys**: Ensure all API keys are correctly set as environment variables
2. **CORS**: Verify that your frontend domain is allowed in the backend CORS configuration
3. **Vector Database**: Check that Qdrant is properly configured and accessible
4. **Embedding Dimensions**: Ensure consistency between embedding models for indexing and querying

### Debugging Tips
- Check backend logs for error messages
- Verify network connectivity between components
- Confirm that book content has been properly indexed in Qdrant
- Test API endpoints individually before testing the full pipeline