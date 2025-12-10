# Agent Context: RAG Chatbot Integration

## Technologies Used

### Backend
- **FastAPI**: Modern Python web framework for building APIs
- **OpenAI Agent SDK**: Framework for creating AI agents with tools
- **Google Generative AI SDK**: Interface for Gemini models
- **Cohere API**: Embedding generation service
- **Qdrant Client**: Vector database client for similarity search
- **Python-dotenv**: Environment variable management

### Frontend
- **ChatKit SDK**: Chat widget for Docusaurus integration
- **JavaScript**: Client-side functionality for text selection and chat

### Deployment
- **Railway**: Backend deployment platform
- **GitHub Pages**: Frontend documentation hosting

## Architecture Pattern

The system follows a RAG (Retrieval Augmented Generation) pattern:
1. Documents are processed and embedded using Cohere
2. Embeddings are stored in Qdrant vector database
3. User queries are converted to embeddings and matched against stored content
4. Relevant content chunks are sent to Gemini via OpenAI Agent SDK
5. Agent generates responses based on retrieved context

## Key Implementation Details

### Agent Tools
- `retrieve_chunks(query)`: Queries Qdrant for relevant content
- `answer_with_context(question, chunks)`: Generates answers from provided context

### API Endpoints
- `POST /embed`: Processes documents and creates embeddings
- `POST /rag`: Performs RAG query with context retrieval
- `POST /rag/selected`: Handles selected-text-only queries
- `POST /api/conversation`: Chat conversation endpoint

### Data Flow
- Text content → Cohere embeddings → Qdrant storage → similarity search → Gemini response

## Environment Variables
- `GEMINI_API_KEY`: API key for Google Gemini service
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `COHERE_API_KEY`: API key for Cohere embeddings

## Best Practices
- Use 1024-dimensional vectors for Cohere embeddings
- Implement proper error handling for all API calls
- Validate embeddings exist before RAG queries
- Handle rate limiting for external API services
- Implement fallback messages when services are unavailable