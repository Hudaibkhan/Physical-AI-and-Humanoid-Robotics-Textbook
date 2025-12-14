# Book RAG Agent

This is a FastAPI application that implements a RAG (Retrieval Augmented Generation) agent using OpenAI Agents SDK with Gemini 2.5 Flash model. The agent can answer questions based on book content stored in Qdrant vector database or from user-selected text.

## Features

- **Normal RAG Mode**: Queries Qdrant vector database for relevant book content
- **Selected Text Mode**: Answers questions based only on user-provided selected text
- **Session Management**: Maintains conversation context across multiple questions
- **Citation Support**: Always cites the source chunks used to generate responses
- **No Hallucination**: Only answers from provided context, never generates information

## Architecture

- **FastAPI**: Web framework for the backend API
- **OpenAI Agents SDK**: Framework for creating AI agents with tools
- **LiteLLM**: Interface for connecting to Gemini model
- **Qdrant**: Vector database for similarity search
- **Python-dotenv**: Environment variable management

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in a `.env` file:
   ```
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=your_collection_name
   ```

3. Run the application:
   ```bash
   uvicorn app:app --reload --port 8000
   ```

## API Endpoints

- `POST /chat`: Main endpoint for chatting with the agent
  - Request body:
    ```json
    {
      "message": "Your question",
      "selected_text": "Optional selected text for focused queries",
      "session_id": "Optional session ID for conversation continuity"
    }
    ```
  - Response:
    ```json
    {
      "response": "Agent's answer",
      "source_chunks": ["List of source chunks used"],
      "session_id": "Session ID",
      "citations": ["List of citations"]
    }
    ```

## Deployment

### Hugging Face Spaces
The backend can be deployed to Hugging Face Spaces as a Docker container. The necessary files are:
- app.py
- agent.py
- connection.py
- requirements.txt
- Dockerfile (if needed)

### Environment Variables
Make sure to configure the following environment variables in your deployment environment:
- GEMINI_API_KEY
- QDRANT_URL
- QDRANT_API_KEY
- QDRANT_COLLECTION_NAME