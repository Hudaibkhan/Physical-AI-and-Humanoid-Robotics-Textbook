# Data Model: Book RAG Agent Upgrade

## Entities

### Book Content
**Description**: Represents the robotics book content stored in Qdrant vector database
- **Attributes**:
  - content (string): The text content of the book section
  - embedding (array of numbers): Vector representation of the content
  - metadata (object): Additional information about the content (source, section, etc.)
  - id (string): Unique identifier for the content chunk

### Agent Session
**Description**: Represents a user's conversation session with the agent
- **Attributes**:
  - session_id (string): Unique identifier for the session
  - conversation_history (array): List of message exchanges
  - created_at (timestamp): When the session was created
  - last_activity (timestamp): When the session was last used

### Query Result
**Description**: Represents the results from the RAG system
- **Attributes**:
  - id (string): Identifier of the source chunk
  - content (string): The text content of the retrieved chunk
  - similarity_score (number): Similarity score between query and chunk (0.0-1.0)
  - metadata (object): Additional information about the source

### Agent Request
**Description**: Represents a request to the agent
- **Attributes**:
  - message (string): The user's question or input
  - selected_text (string | null): Optional text selected by the user
  - session_id (string | null): Session identifier for conversation context
  - mode (string): "rag" or "selected" indicating the retrieval mode

### Agent Response
**Description**: Represents the agent's response to a user request
- **Attributes**:
  - response (string): The agent's answer to the user's question
  - source_chunks (array): List of Query Result objects used to generate the response
  - session_id (string): Session identifier for conversation continuity
  - citations (array): References to the source chunks used