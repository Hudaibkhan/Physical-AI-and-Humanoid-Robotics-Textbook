# Agent Context: Book RAG Agent Upgrade

## Technologies Used

### Backend
- **FastAPI**: Modern Python web framework for building APIs
- **OpenAI Agents SDK**: Framework for creating AI agents with tools (upgraded from OpenAI Agent SDK)
- **LiteLLM**: Interface for connecting to various LLM providers including Gemini
- **Google Generative AI SDK**: Interface for Gemini models
- **Cohere API**: Embedding generation service
- **Qdrant Client**: Vector database client for similarity search
- **Python-dotenv**: Environment variable management

### Frontend
- **ChatKit SDK**: Chat widget for Docusaurus integration
- **JavaScript**: Client-side functionality for text selection and chat

### Deployment
- **Hugging Face Spaces**: Backend deployment platform
- **Vercel**: Frontend deployment platform (updated from GitHub Pages)

## Architecture Pattern

The system follows a RAG (Retrieval Augmented Generation) pattern with OpenAI Agents SDK:
1. Documents are processed and embedded using Cohere or Gemini embeddings
2. Embeddings are stored in Qdrant vector database
3. User queries are processed by the OpenAI Agent with a custom rag_query tool
4. The rag_query tool handles two modes:
   - "rag" mode: Queries Qdrant for relevant content
   - "selected" mode: Processes user-selected text for focused queries
5. Retrieved content chunks are sent to Gemini 2.5 Flash via LiteLLM
6. Agent generates responses based on retrieved context and cites sources

## Key Implementation Details

### Agent Components
- `book_rag_agent`: Main agent that processes queries using OpenAI Agents SDK
- `rag_query(query: str, mode: str, top_k: int)`: Function tool for retrieval with two modes
  - mode="rag": Retrieve from Qdrant vector database
  - mode="selected": Retrieve from user-selected text chunks

### API Endpoints
- `POST /chat`: Main chat endpoint accepting message, selected_text, and session_id
- `POST /session`: Create new agent session (if needed)

### Data Flow
- Text content → Cohere/Gemini embeddings → Qdrant storage → similarity search → LiteLLM → Gemini 2.5 Flash response

## Environment Variables
- `GEMINI_API_KEY`: API key for Google Gemini service
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `COHERE_API_KEY`: API key for Cohere embeddings

## Best Practices
- Use OpenAI Agents SDK with LiteLLM for external model integration
- Implement proper error handling for all API calls
- Validate embeddings exist before RAG queries
- Handle rate limiting for external API services
- Implement fallback messages when services are unavailable
- Ensure consistent response formatting between RAG and selected text modes
- Always cite source chunks in agent responses
- Never allow the agent to hallucinate information outside the provided context