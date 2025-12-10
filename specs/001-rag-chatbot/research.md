# Research: RAG Chatbot for Digital Book Website

## Overview
This research document addresses the technical decisions and unknowns identified during the planning phase for the RAG chatbot project.

## Technology Stack Decisions

### Decision: Backend Framework - FastAPI
**Rationale**: FastAPI is chosen for the backend due to its high performance, built-in async support, automatic API documentation generation, and strong typing with Pydantic. It's well-suited for ML/AI applications and has excellent integration with the Python ecosystem required for this project.

**Alternatives considered**:
- Flask: More established but slower and lacks automatic documentation
- Django: Too heavy for this API-focused application
- Node.js/Express: Would require different language skills and ecosystem

### Decision: Vector Database - Qdrant Cloud Free Tier
**Rationale**: Qdrant is specifically designed for vector similarity search and has excellent Python client support. The free tier provides sufficient capacity for this project and integrates well with the embedding models we're using.

**Alternatives considered**:
- Pinecone: More expensive and less open-source friendly
- Weaviate: Good alternative but Qdrant has better free tier for this use case
- FAISS: Local solution but lacks cloud scalability

### Decision: LLM - Google Gemini
**Rationale**: Google Gemini is explicitly required by the specification and offers excellent reasoning capabilities for RAG applications. The API is well-documented and reliable.

**Alternatives considered**:
- OpenAI GPT: Would require different API key and potentially different integration
- Anthropic Claude: Would require different API integration

### Decision: Embedding Model - Gemini Embeddings
**Rationale**: Using Gemini embeddings provides consistency with the LLM choice and is likely optimized to work well with Gemini for RAG applications. It's also available as a free tier option.

**Alternatives considered**:
- OpenAI embeddings: Would require additional API key and different integration
- Sentence Transformers: Open-source but would require hosting and could be slower

### Decision: Frontend Integration - JavaScript Widget
**Rationale**: A JavaScript widget provides the flexibility needed to integrate with the existing Docusaurus site without requiring changes to the core book structure. It can be loaded via a script tag as specified in the requirements.

**Alternatives considered**:
- React component: Would require more integration with Docusaurus build process
- iframe: Would create isolation but potentially styling/communication issues

## Architecture Decisions

### Decision: Separate Backend and Frontend Architecture
**Rationale**: Separating the backend (FastAPI) and frontend (JavaScript widget) provides clear separation of concerns, allows independent scaling, and enables the backend to be reused for other applications.

**Alternatives considered**:
- Monolithic application: Would create tight coupling and deployment complexity
- Server-side rendering: Not needed for this use case

### Decision: Text Chunking Strategy
**Rationale**: For Markdown book content, we'll use a strategy that respects document structure (paragraphs, sections) while creating chunks of appropriate size for embedding models (typically 512-1024 tokens). This preserves context while enabling efficient retrieval.

**Alternatives considered**:
- Fixed character count: Might break up meaningful content
- Sentence-based: Could create very short chunks for books with short sentences

## API Design Considerations

### Decision: REST API with JSON
**Rationale**: A REST API with JSON payloads provides simplicity, broad compatibility, and clear documentation. This is appropriate for the RAG chatbot functionality.

**Alternatives considered**:
- GraphQL: More complex than needed for this use case
- gRPC: Would require additional tooling and complexity

## Deployment Strategy

### Decision: Render for Backend, Vercel for Frontend
**Rationale**: Render provides good Python/ FastAPI support with a free tier, while Vercel is optimized for static site hosting and JavaScript applications. This matches the deployment requirements specified.

**Alternatives considered**:
- AWS/GCP: More complex setup for free tier usage
- Heroku: No longer offers free tier

## Security Considerations

### Decision: API Key Management
**Rationale**: API keys for Gemini, Qdrant, and embedding services will be managed through environment variables in the backend. The frontend will not have direct access to these keys, ensuring security.

**Alternatives considered**:
- Client-side keys: Would expose sensitive information
- Proxy approach: Chosen approach provides necessary security

## Performance Considerations

### Decision: Caching Strategy
**Rationale**: For frequently asked questions, we may implement a caching layer to reduce LLM API calls and improve response times. Redis or in-memory caching could be used.

**Alternatives considered**:
- No caching: Would result in higher costs and slower responses for repeated queries
- Client-side caching: Would be less secure and harder to manage consistency