# Research Document: RAG Chatbot Integration Fix + Agent SDK Migration

**Feature**: RAG Chatbot Integration Fix + Agent SDK Migration
**Date**: 2025-12-10
**Status**: Completed

## Research Objectives

This document addresses the unknowns and dependencies identified in the technical context of the implementation plan, specifically focusing on:

1. OpenAI Agent SDK with Gemini API integration
2. MCP tools integration for RAG actions
3. ChatKit widget integration with OpenAI Agent
4. Qdrant collection schema for Cohere embeddings

## Research Findings

### 1. OpenAI Agent SDK with Gemini API Integration

**Decision**: Use Google's Vertex AI or Google AI SDK to integrate Gemini with OpenAI Agent SDK

**Rationale**: The OpenAI Agent SDK is specifically designed for OpenAI's models and APIs. To use Gemini as the LLM provider, we need to create a custom integration that either:
- Uses Google's Vertex AI SDK to interface with Gemini
- Creates a custom LLM provider wrapper that conforms to OpenAI's interface
- Uses the Google AI SDK to access Gemini models

**Implementation Approach**:
- Create a custom LLM provider that implements the same interface as OpenAI's models
- Use the Google AI SDK (google-generativeai) to interface with Gemini
- Configure the OpenAI Agent SDK to use this custom provider

**Code Pattern**:
```python
import google.generativeai as genai
from openai import OpenAI

class GeminiLLMProvider:
    def __init__(self, api_key):
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-pro')

    def chat_completions_create(self, messages, **kwargs):
        # Convert OpenAI format to Gemini format
        prompt = self._convert_messages_to_prompt(messages)
        response = self.model.generate_content(prompt)
        return self._convert_to_openai_format(response)
```

**Alternatives Considered**:
- Using OpenAI-compatible API proxy services (less reliable)
- Direct API calls without Agent SDK (loses agent functionality)
- Custom agent implementation (more complex than needed)

### 2. MCP Tools Integration for RAG Actions

**Decision**: Use Anthropic's Message Creation Protocol (MCP) tools as specified in context7 guidelines

**Rationale**: MCP (Message Creation Protocol) is a standard for defining tools that can be used by AI agents. For RAG actions, we need to create specific tools that can be called by the agent when needed.

**Implementation Approach**:
- Create `retrieve_chunks` tool that queries Qdrant
- Create `answer_with_context` tool that generates responses
- Define tools using MCP format as documented in context7

**Tool Definition Pattern**:
```python
def retrieve_chunks(query: str) -> list:
    """
    Retrieve relevant text chunks from the vector database based on the query.

    Args:
        query: The search query to find relevant chunks

    Returns:
        List of text chunks that are relevant to the query
    """
    # Implementation to query Qdrant
    pass

def answer_with_context(question: str, chunks: list) -> str:
    """
    Generate an answer to the question based on the provided context chunks.

    Args:
        question: The question to answer
        chunks: List of context chunks to use for answering

    Returns:
        Generated answer based on the context
    """
    # Implementation to generate answer with context
    pass
```

### 3. ChatKit Widget Integration with OpenAI Agent

**Decision**: Use ChatKit's custom agent integration capability

**Rationale**: ChatKit supports custom AI agents through its API integration. We need to connect the ChatKit frontend to our backend endpoints that interface with the OpenAI Agent.

**Implementation Approach**:
- Configure ChatKit to use custom API endpoints
- Create backend endpoints that translate ChatKit requests to OpenAI Agent calls
- Handle streaming responses for real-time chat experience

**Integration Pattern**:
```javascript
// In Docusaurus page
<script src="https://cdn.chatkit.com/chatkit.js"></script>
<script>
  ChatKit.init({
    agentId: "your-agent-id",
    apiEndpoint: "https://your-backend/api/conversation",
    // Additional configuration
  });
</script>
```

**Alternatives Considered**:
- Building custom chat widget (more work)
- Using different chat SDK (requires different learning curve)
- Direct OpenAI API integration (less flexible)

### 4. Qdrant Collection Schema for Cohere Embeddings

**Decision**: Use 1024-dimensional vectors with cosine distance for Cohere embeddings

**Rationale**: Cohere's multilingual embedding model (v3) produces 1024-dimensional vectors. Using cosine distance is standard for embedding similarity.

**Schema Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Collection configuration
collection_config = models.VectorParams(
    size=1024,  # Cohere embedding dimension
    distance=models.Distance.COSINE
)

# Payload schema for book chunks
payload_schema = {
    "content": "text",
    "source_file": "keyword",
    "chunk_index": "integer",
    "metadata": "keyword"
}
```

**Vector Dimensions by Cohere Model**:
- Cohere embed-english-v3.0: 1024 dimensions
- Cohere embed-multilingual-v3.0: 1024 dimensions
- Using multilingual as it supports multiple languages

## Best Practices Identified

### 1. Error Handling
- Implement comprehensive error handling for all external API calls
- Provide graceful fallbacks when services are unavailable
- Log errors for debugging while maintaining user privacy

### 2. Rate Limiting
- Implement proper rate limiting for API calls to Cohere, Qdrant, and Gemini
- Use caching to minimize redundant API calls
- Handle rate limit responses appropriately

### 3. Security
- Secure API keys using environment variables
- Implement proper authentication if needed
- Validate all user inputs to prevent injection attacks

### 4. Performance
- Use asynchronous processing where possible
- Implement efficient chunking strategies
- Optimize vector searches with proper indexing

## Implementation Recommendations

1. **Start with Backend**: Implement the FastAPI backend with Qdrant integration first
2. **Test Embedding Pipeline**: Ensure the embedding pipeline works correctly before proceeding
3. **Agent Integration**: Focus on getting the OpenAI Agent SDK working with custom LLM provider
4. **Frontend Integration**: Connect ChatKit to the backend endpoints last
5. **Iterative Testing**: Test each component individually before integration

## Risks and Mitigation

### 1. API Compatibility Risk
**Risk**: OpenAI Agent SDK may not work well with custom LLM providers
**Mitigation**: Thoroughly test the custom LLM provider implementation and have fallback options

### 2. Performance Risk
**Risk**: Vector searches may be slow with large datasets
**Mitigation**: Implement proper indexing in Qdrant and optimize chunk sizes

### 3. Deployment Risk
**Risk**: Multiple API keys and services may complicate deployment
**Mitigation**: Use proper configuration management and environment variable handling

## Conclusion

All identified unknowns have been researched and resolved:

1. OpenAI Agent SDK can be used with Gemini through a custom LLM provider wrapper
2. MCP tools can be implemented following standard patterns for RAG actions
3. ChatKit can integrate with custom agents through API endpoints
4. Qdrant should use 1024-dimensional vectors with cosine distance for Cohere embeddings

The research provides a solid foundation for implementing the RAG chatbot with the required technologies while maintaining good practices for security, performance, and maintainability.