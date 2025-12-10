# Quickstart Guide: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration Fix + Agent SDK Migration
**Date**: 2025-12-10

## Overview

This guide provides step-by-step instructions to set up and run the RAG chatbot system with OpenAI Agent SDK, Qdrant vector database, Cohere embeddings, and Gemini API.

## Prerequisites

- Python 3.9 or higher
- pip package manager
- Git
- API keys for:
  - Qdrant Cloud (QDRANT_URL and QDRANT_API_KEY)
  - Cohere (COHERE_API_KEY)
  - Gemini (GEMINI_API_KEY)

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Set up Python Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 3. Install Dependencies

```bash
pip install fastapi uvicorn python-dotenv openai cohere qdrant-client google-generativeai pydantic
```

### 4. Configure Environment Variables

Create a `.env` file in the project root:

```bash
cp .env.example .env
```

Edit `.env` and add your API keys:

```env
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
```

### 5. Run the Embedding Pipeline

Before using the RAG functionality, you need to process your textbook content:

```bash
python scripts/embed_pipeline.py --source-path "/path/to/your/markdown/files"
```

This will:
- Load markdown files from the specified path
- Chunk the text into appropriate sizes
- Generate embeddings using Cohere
- Store the embeddings in Qdrant collection "book_chunks"

### 6. Start the Backend Server

```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

## API Usage Examples

### 1. Perform RAG Query

```bash
curl -X POST http://localhost:8000/rag \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is humanoid robotics?"
  }'
```

### 2. Selected Text RAG

```bash
curl -X POST http://localhost:8000/rag/selected \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "Humanoid robotics is a field that focuses on creating robots with human-like characteristics.",
    "query": "What does this text say about humanoid robotics?"
  }'
```

### 3. Chat Conversation

```bash
curl -X POST http://localhost:8000/api/conversation \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Can you explain the basics of humanoid robotics?",
    "session_id": "optional-session-id"
  }'
```

## Frontend Integration

### 1. Docusaurus Integration

Add the ChatKit script to your Docusaurus configuration:

In `docusaurus.config.js`, add to the `headTags`:

```javascript
headTags: [
  {
    tagName: 'script',
    attributes: {
      src: 'https://cdn.chatkit.com/chatkit.js',
      async: true,
    },
  },
],
```

### 2. Widget Initialization

Add this script to your Docusaurus pages to initialize the chat widget:

```html
<script>
  document.addEventListener('DOMContentLoaded', function() {
    if (window.ChatKit) {
      ChatKit.init({
        agentId: "your-agent-id",
        apiEndpoint: "http://localhost:8000/api/conversation",
        containerId: "chatkit-container",
        // Additional configuration options
      });
    } else {
      console.error("ChatKit failed to load");
    }
  });
</script>
```

## Testing the System

### 1. Verify Embeddings Exist

```bash
curl -X GET http://localhost:8000/health
```

### 2. Test RAG Query

```bash
curl -X POST http://localhost:8000/rag \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Test query to verify RAG is working"
  }'
```

### 3. Check Qdrant Connection

Verify that the "book_chunks" collection exists in your Qdrant instance and contains the expected number of vectors.

## Troubleshooting

### Common Issues

1. **Embedding Not Found Error**
   - Solution: Run the embedding pipeline first
   - Command: `python scripts/embed_pipeline.py --source-path "/path/to/markdown"`

2. **API Key Errors**
   - Solution: Verify all API keys are correctly set in `.env` file
   - Check: QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, GEMINI_API_KEY

3. **Qdrant Connection Issues**
   - Solution: Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Check: Network connectivity to Qdrant Cloud

4. **CORS Errors**
   - Solution: Configure CORS in your FastAPI application
   - Add: CORS middleware to allow requests from your frontend domain

### Debugging Tips

- Check the console logs when running the server with `--reload` flag
- Verify environment variables are loaded: `print(os.getenv('GEMINI_API_KEY'))`
- Test individual API endpoints using curl or Postman
- Monitor Qdrant dashboard for collection status and health

## Next Steps

1. Deploy the backend to Railway or similar platform
2. Integrate the frontend widget into your Docusaurus site
3. Test the complete end-to-end functionality
4. Optimize chunk sizes and embedding parameters based on your content
5. Add monitoring and logging for production use