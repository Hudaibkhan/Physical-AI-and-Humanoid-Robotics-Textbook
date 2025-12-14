# Quickstart Guide: Book RAG Agent

## Overview
This guide provides instructions to quickly set up and run the Book RAG Agent using OpenAI Agents SDK with Gemini 2.5 Flash model.

## Prerequisites
- Python 3.9+
- Access to Google Gemini API
- Qdrant vector database access
- Node.js (for frontend deployment)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup
```bash
# Navigate to backend directory
cd fastapi_app

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Environment Configuration
Create a `.env` file in the backend directory with the following variables:
```env
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
```

### 4. Run the Backend
```bash
# From the fastapi_app directory
uvicorn app:app --reload --port 8000
```

## Usage

### 1. API Interaction
Send a POST request to `/chat` endpoint:

**Normal RAG Mode:**
```json
{
  "message": "What are the key principles of humanoid robotics?",
  "selected_text": null,
  "session_id": "sess_abc123xyz"
}
```

**Selected Text Mode:**
```json
{
  "message": "Explain the control systems mentioned here?",
  "selected_text": "The control systems in humanoid robotics typically involve...",
  "session_id": "sess_abc123xyz"
}
```

### 2. Response Format
The agent will respond with:
- The answer to your question
- Source citations
- Relevant chunks used to generate the response

## Frontend Integration

### 1. ChatKit Setup
Integrate the ChatKit widget with your frontend by including the script that connects to your backend's `/chat` endpoint.

### 2. Environment Configuration
Configure the frontend to connect to your deployed backend API endpoint.

## Running Tests
```bash
# From the backend directory
python -m pytest tests/
```

## Deployment
### Hugging Face Spaces (Backend)
1. Create a Space with Docker
2. Add your backend files (app.py, agent.py, connection.py, requirements.txt)
3. Configure environment variables in Space settings

### Vercel (Frontend)
1. Deploy your Docusaurus site with ChatKit integration
2. Configure the frontend to connect to your Hugging Face backend