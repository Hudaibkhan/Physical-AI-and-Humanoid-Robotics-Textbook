# Research Document: Book RAG Agent Upgrade

## Decision: OpenAI Agents SDK Implementation Approach
**Rationale**: Based on the context7 documentation, the OpenAI Agents SDK supports external models like Gemini 2.5 Flash through LiteLLM integration. This allows us to use the familiar OpenAI-style API while connecting to Google's Gemini model.

**Implementation Details**:
- Use `LitellmModel` for Gemini 2.5 Flash integration
- Register function tools using the `@function_tool` decorator
- Use `Runner.run()` to execute agent operations
- Implement the `rag_query` function tool with parameters for query, mode, and top_k

## Decision: External Model Integration
**Rationale**: The OpenAI Agents SDK supports external models via LiteLLM by prefixing the model identifier with 'litellm/'. This approach allows us to use Gemini 2.5 Flash while maintaining compatibility with the OpenAI Agents SDK.

**Implementation Details**:
- Model identifier: `litellm/gemini/gemini-2.5-flash`
- Requires proper API key configuration for Google Gemini
- May need to implement a custom model configuration using `LitellmModel`

## Decision: Function Tool Implementation
**Rationale**: The `@function_tool` decorator is the standard way to register tools with the OpenAI Agents SDK. This allows the agent to call specific functions during its execution.

**Implementation Details**:
- Create `rag_query(query: str, mode: str, top_k: int)` function with the `@function_tool` decorator
- Implement different logic paths for "rag" mode (Qdrant) and "selected" mode (selected text)
- Return structured data with id, text, and similarity score

## Decision: Qdrant Integration
**Rationale**: We'll leverage the existing Qdrant integration from the current RAG system but adapt it to work with the OpenAI Agents SDK function tools.

**Implementation Details**:
- Reuse existing Qdrant client configuration
- Adapt existing search functionality to work within the `rag_query` function tool
- Maintain the same vector dimensions and similarity scoring

## Decision: Selected Text Processing
**Rationale**: For the selected text mode, we need to implement a different retrieval mechanism that processes user-provided text rather than searching the Qdrant database.

**Implementation Details**:
- Implement text chunking for selected text
- Create embeddings for the selected text chunks
- Perform similarity search between query and selected text chunks
- Return relevant chunks with similarity scores

## Decision: FastAPI Backend Structure
**Rationale**: The backend needs to serve as an interface between the ChatKit frontend and the OpenAI Agents SDK.

**Implementation Details**:
- Create `/chat` endpoint that accepts message, selected_text, and session_id
- Determine mode based on presence of selected_text
- Pass parameters to the agent execution
- Return structured responses with citations

## Decision: Session Management
**Rationale**: To maintain conversation context, we need to implement session management that works with the OpenAI Agents SDK.

**Implementation Details**:
- Reuse existing session management approach if compatible
- Pass session context to the agent if needed
- Maintain conversation history for context awareness