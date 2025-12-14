# Feature Specification: Book RAG Agent Upgrade

**Feature Branch**: `003-book-rag-agent-upgrade`
**Created**: 2025-12-12
**Status**: Draft
**Input**: User description: "-Book RAG Agent Upgrade Specification

This specification defines how Claude Code + SpecKit Plus must generate and update the RAG Chatbot system using OpenAI Agents SDK, Gemini model provider, Qdrant, FastAPI backend, and ChatKit UI.

## 🎯 Goal

Upgrade the existing chatbot so that:

It is rebuilt using the OpenAI Agents SDK

Agent uses Gemini 2.5 Flash as the external model provider

All knowledge comes only from the robotics book stored in Qdrant

Supports:

Normal RAG mode

Selected Text RAG mode

Uses MCP + context7 to fetch OpenAI Agents SDK documentation before generating code

Frontend uses ChatKit

Backend is FastAPI deployed on Hugging Face Spaces

## 1. MCP + Context7 Requirements

Before building anything, Claude Code must:

context7 - resolve-library-id (MCP)(libraryName: \"OpenAI Agents SDK\")

This MCP tool returns:

Official documentation

Function-tool schema

How to attach external model providers

How to define tools

Best practices for agent memory, run configuration, sessions, etc.

Claude Code MUST use the docs returned by context7 to generate correct agent code.

## 2. Agent Requirements
2.1 Agent Name
book_rag_agent

2.2 Model

Model must always be:

Gemini 2.5 Flash


via OpenAI-style external provider wrapper.

2.3 Instructions

Agent must:

ONLY answer from book content (Qdrant dataset or selected text)

NEVER hallucinate

ALWAYS cite retrieved chunks

ALWAYS call RAG tool before answering

Ignore any external world knowledge

2.4 Tools

Agent must expose ONE function tool:

rag_query(query: str, mode: str, top_k: number)


If mode = \"rag\" → retrieve from Qdrant

If mode = \"selected\" → retrieve from user-selected text chunks

Return:

id

text

similarity score

2.5 Retrieval Logic
Normal RAG mode

Convert user query to embedding

Retrieve top_k similar chunks from Qdrant

Agent uses these chunks to answer

Selected Text mode

Split provided text into small chunks

Compute vector similarity between query and selected text chunks

Return the closest chunks

Agent uses ONLY these chunks

## 3. File Architecture
fastapi_app/
  ├── connection.py
  │     → Create Gemini external provider client
  │     → Create embeddings client
  │     → Create Qdrant client
  │     → Utility: embed(), qdrant_search(), selected_text_search()
  │
  ├── agent.py
  │     → Define rag_query() function tool
  │     → Build book_rag_agent with OpenAI Agents SDK
  │     → Configure model provider using connection.py
  │
  ├── app.py
  │     → FastAPI entry
  │     → POST /chat endpoint
  │     → Pass message + selected_text to agent
  │     → Run agent using Runner.run()
  │     → Return final_output

## 4. FastAPI Backend Requirements
Endpoint

POST /chat

Body
{
  \"message\": \"user message\",
  \"selected_text\": \"... | null\",
  \"session_id\": \"uuid | null\"
}

Logic
if selected_text:
    rag_mode = \"selected\"
else:
    rag_mode = \"rag"


Then:

await Runner.run(agent, message, run_config=config, metadata={...})

## 5. Frontend (ChatKit) Requirements

ChatKit widget must:

Allow users to select text in the book

Send selected text to backend

Maintain message history

Display citations under answers

## 6. Deployment Requirements
Hugging Face (Backend)

Files needed:

app.py

agent.py

connection.py

requirements.txt

Vercel (Frontend)

ChatKit widget installed in Docusaurus

Calls HF backend /chat

## 7. Cleanup Requirement

Claude Code must delete all useless, previous chatbot files

Project must be cleaned before generating new code

## 8. Output That Claude Code Must Produce

After this /sp.specify, Claude Code must generate:

1. Updated project structure
2. connection.py
3. agent.py
4. app.py
5. ChatKit integration script
6. HF deployment config"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Basic Book Query with OpenAI Agents (Priority: P1)

As a user, I want to ask questions about the robotics book content and get accurate answers from the RAG system using the OpenAI Agents SDK with Gemini 2.5 Flash model, so that I can learn about robotics concepts effectively.

**Why this priority**: This is the core functionality that delivers the primary value of the RAG system - providing accurate answers based on the book content.

**Independent Test**: Can be fully tested by asking a question about the book content and verifying that the agent responds with accurate information from the book, citing the relevant chunks.

**Acceptance Scenarios**:

1. **Given** the agent is initialized with the book content in Qdrant, **When** I ask a question about robotics concepts, **Then** the agent retrieves relevant chunks from Qdrant and provides an accurate answer citing the source chunks
2. **Given** the agent has access to book content, **When** I ask a question not covered by the book, **Then** the agent responds with "This information is not in the book."

---

### User Story 2 - Selected Text Query with OpenAI Agents (Priority: P2)

As a user, I want to select specific text in the book and ask questions about only that selected text, so that I can get focused answers based on my specific selection rather than the entire book.

**Why this priority**: This provides an enhanced user experience by allowing focused queries on specific content sections.

**Independent Test**: Can be tested by providing selected text and asking a question, verifying that the agent responds based only on the selected text rather than searching the entire book.

**Acceptance Scenarios**:

1. **Given** I have selected text from the book, **When** I ask a question with the selected text context, **Then** the agent uses only the selected text to generate the answer

---

### User Story 3 - Chat History and Session Management (Priority: P3)

As a user, I want to maintain conversation context across multiple questions in a session, so that I can have a natural conversation with the agent about the book content.

**Why this priority**: This enhances user experience by allowing contextual conversations rather than isolated queries.

**Independent Test**: Can be tested by asking follow-up questions in the same session and verifying that the agent maintains context from previous exchanges.

**Acceptance Scenarios**:

1. **Given** I'm in an active session, **When** I ask a follow-up question that references previous context, **Then** the agent maintains conversation context and responds appropriately

---

### Edge Cases

- What happens when Qdrant is unavailable or returns no results?
- How does the system handle very long selected text that exceeds model token limits?
- How does the system handle queries in languages other than what's in the book?
- What happens when the agent encounters a query it cannot answer based on the provided context?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST use OpenAI Agents SDK to orchestrate the book Q&A agent
- **FR-002**: System MUST use Gemini 2.5 Flash as the LLM provider via LiteLLM integration
- **FR-003**: Agent MUST retrieve relevant chunks from Qdrant vector database when in normal RAG mode
- **FR-004**: Agent MUST retrieve relevant chunks from user-selected text when in selected text mode
- **FR-005**: Agent MUST ALWAYS cite the retrieved chunks in its responses
- **FR-006**: Agent MUST NOT hallucinate information outside of the provided context (book content or selected text)
- **FR-007**: System MUST expose a rag_query function tool with parameters (query: str, mode: str, top_k: number)
- **FR-008**: System MUST handle both "rag" and "selected" modes for the rag_query tool
- **FR-009**: System MUST provide a FastAPI backend with a POST /chat endpoint
- **FR-010**: POST /chat endpoint MUST accept message, selected_text, and session_id in request body
- **FR-011**: System MUST return agent responses with source chunk citations
- **FR-012**: Frontend MUST integrate ChatKit widget for user interactions
- **FR-013**: System MUST support session management for maintaining conversation context
- **FR-014**: System MUST compute embeddings for selected text when in selected text mode

### Key Entities *(include if feature involves data)*

- **Book Content**: Represents the robotics book content stored in Qdrant vector database, with attributes including content text, embedding vector, and metadata
- **Agent Session**: Represents a user's conversation session with the agent, with attributes including session ID, conversation history, and context
- **Query Result**: Represents the results from the RAG system, with attributes including retrieved chunks, similarity scores, and source citations

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can ask questions about robotics book content and receive accurate answers based on the book content within 5 seconds
- **SC-002**: Agent responses include citations to the specific book chunks used to generate the answer 100% of the time
- **SC-003**: System correctly handles both normal RAG mode and selected text mode with 95% accuracy
- **SC-004**: 90% of user questions about book content receive relevant, accurate answers based on the book content
- **SC-005**: System successfully integrates with Qdrant vector database and retrieves relevant content with 90% precision