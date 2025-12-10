# Feature Specification: RAG Chatbot Integration Fix + Agent SDK Migration

**Feature Branch**: `002-rag-chatbot-migration`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "RAG Chatbot Integration Fix + Agent SDK Migration

REQ-001: Agent Creation Using OpenAI Agent SDK
  - The RAG chatbot agent MUST be implemented using the official OpenAI Agent SDK.
  - Agent MUST expose a function/tool for:
      - retrieve_chunks(query) → fetch from Qdrant
      - answer_with_context(question, chunks
  - Gemini API MUST be injected inside the agent as the LLM provider.
  - MCP MUST be used as documented in context7 (agent-tools).
  - All unofficial or custom agent code MUST be removed.

REQ-002: Embeddings Verification (Cohere Free Model)
  - Embeddings MUST be generated using Cohere free embedding model.
  - System MUST verify that embeddings exist before performing any Qdrant query.
  - If embeddings are missing, backend MUST show: "Embedding not found: Re‑run embed pipeline."

REQ-003: Qdrant Integration
  - Backend MUST connect to Qdrant Cloud Free Tier using:
        QDRANT_URL
        QDRANT_API_KEY
  - A collection MUST exist (name: "book_chunks").
  - Backend MUST provide FastAPI routes:
        POST /embed → create embeddings + upsert to Qdrant
        POST /rag → retrieve + generate
        POST /rag/selected → selected-text-only RAG

REQ-004: ChatKit Frontend UI
  - Chatbot widget MUST use ChatKit frontend SDK.
  - The widget MUST render the agent conversation UI and support streaming.
  - The widget MUST load the agent you created via agent ID.
  - All previous unoffical UI/jQuery/chatbot scripts MUST be deleted.

REQ-005: Widget Visibility and Accessibility
  - Widget MUST appear on every page of the Docusaurus book.
  - It MUST show as a floating button in the bottom-right corner.
  - Button MUST include ARIA label: "Open chatbot".
  - If backend or agent fails to load, widget MUST show fallback:
        "Chatbot is unavailable. Please try again later."

REQ-006: Selected Text RAG Mode
  - User selects text → frontend sends it to `/rag/selected`.
  - Agent MUST answer *only* from that selected text.
  - This mode MUST bypass Qdrant.

REQ-007: File Cleanup
  - All old files related to the previous failed chatbot must be deleted, including:
       - old /chatbox.js
       - unused css/chatbot.css
       - old agent helper scripts
       - any API routes not used by the new agent system

REQ-008: Display Fix
  - Chatbot MUST correctly render inside the book pages.
  - No console errors must occur during initialization.
  - ChatKit MUST successfully connect to the OpenAI Agent endpoint.

Acceptance Criteria:
  - Agent appears inside the book UI
  - Agent uses OpenAI Agent SDK + Gemini LLM
  - Chunk retrieval works from Qdrant
  - Selected-text RAG works
  - Widget always visible
  - Embeddings verified
  - No duplicate or unused files"

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

### User Story 1 - Basic Chatbot Interaction (Priority: P1)

As a user reading the physical AI and humanoid robotics textbook, I want to interact with an AI chatbot that can answer questions about the book content using RAG (Retrieval Augmented Generation). I should be able to type questions and receive accurate responses based on the textbook content.

**Why this priority**: This is the core functionality that delivers immediate value to users - being able to ask questions and get relevant answers from the book content.

**Independent Test**: Can be fully tested by asking the chatbot a question and verifying it provides a relevant response based on the book content, delivering the primary value proposition of the feature.

**Acceptance Scenarios**:

1. **Given** I am viewing the textbook page, **When** I click the chatbot button and ask a question about the content, **Then** the chatbot provides a relevant answer based on the book content
2. **Given** I have a question about the textbook content, **When** I type my question in the chatbot interface, **Then** the system retrieves relevant chunks from the book and generates a contextual response

---

### User Story 2 - Selected Text RAG Mode (Priority: P2)

As a user reading the textbook, I want to select specific text and ask questions about just that text, so the chatbot can provide focused answers based only on my selected content rather than the entire book.

**Why this priority**: This provides a more focused and precise way to interact with the content, allowing users to get answers specifically about text they've highlighted.

**Independent Test**: Can be tested by selecting text in the book, asking a question about it, and verifying the chatbot responds based only on the selected text without referencing other parts of the book.

**Acceptance Scenarios**:

1. **Given** I have selected text in the textbook, **When** I right-click or use a context menu to ask the chatbot about the selection, **Then** the system sends the selected text to the `/rag/selected` endpoint and the chatbot responds based only on that text
2. **Given** I have selected text in the book, **When** I trigger the selected text RAG mode, **Then** the chatbot bypasses Qdrant and answers only from the provided text

---

### User Story 3 - Embeddings and Content Indexing (Priority: P3)

As a content administrator, I want to be able to generate embeddings for the textbook content and index it in Qdrant, so that users can effectively search and retrieve relevant information.

**Why this priority**: This is foundational infrastructure that enables the chatbot functionality but can be developed independently before the user-facing features.

**Independent Test**: Can be tested by running the embed pipeline and verifying that content is properly indexed in Qdrant, delivering the indexing capability as a standalone feature.

**Acceptance Scenarios**:

1. **Given** new or updated textbook content exists, **When** I trigger the `/embed` endpoint, **Then** embeddings are generated using Cohere and content is upserted to the Qdrant collection
2. **Given** the system needs to verify embeddings exist, **When** a query is made before embeddings are ready, **Then** the system shows "Embedding not found: Re-run embed pipeline"

---

### Edge Cases

- What happens when Qdrant is unavailable or the API key is invalid?
- How does the system handle missing or malformed embeddings?
- What occurs when the chatbot widget fails to load due to network issues?
- How does the system behave when the selected text is too long or too short?
- What happens when there are no relevant chunks found for a query?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST implement the RAG chatbot agent using the official OpenAI Agent SDK
- **FR-002**: Agent MUST expose a tool/function for retrieving content chunks from Qdrant by query
- **FR-003**: Agent MUST expose a tool/function for answering questions with context from provided chunks
- **FR-004**: System MUST use Gemini API as the LLM provider within the agent
- **FR-005**: System MUST verify that embeddings exist before performing any Qdrant query
- **FR-006**: System MUST connect to Qdrant Cloud Free Tier using QDRANT_URL and QDRANT_API_KEY
- **FR-007**: System MUST create and maintain a Qdrant collection named "book_chunks"
- **FR-008**: System MUST provide a FastAPI endpoint POST /embed that creates embeddings and upserts to Qdrant
- **FR-009**: System MUST provide a FastAPI endpoint POST /rag that retrieves chunks and generates responses
- **FR-010**: System MUST provide a FastAPI endpoint POST /rag/selected for selected-text-only RAG
- **FR-011**: Frontend MUST implement the chatbot widget using ChatKit SDK
- **FR-012**: Widget MUST appear as a floating button in the bottom-right corner on every Docusaurus page
- **FR-013**: Widget button MUST include ARIA label "Open chatbot" for accessibility
- **FR-014**: System MUST handle selected text RAG mode by bypassing Qdrant and using only provided text
- **FR-015**: System MUST show fallback message "Chatbot is unavailable. Please try again later." when backend fails to load
- **FR-016**: System MUST generate embeddings using Cohere free embedding model
- **FR-017**: System MUST remove all previous unofficial chatbot UI, CSS, and script files
- **FR-018**: System MUST ensure no console errors occur during chatbot initialization

### Key Entities *(include if feature involves data)*

- **Book Content Chunks**: Represent segments of the textbook content that have been processed into embeddings for retrieval
- **Qdrant Collection**: A vector database collection named "book_chunks" containing embedded text segments with metadata
- **Chat Session**: Represents a conversation between a user and the AI assistant with message history
- **Embeddings**: Vector representations of text content generated using Cohere's embedding model

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can successfully interact with the chatbot and receive relevant answers to their questions about the textbook content within 5 seconds
- **SC-002**: 95% of user questions receive relevant responses based on the textbook content
- **SC-003**: The selected text RAG mode works correctly, with 90% of responses being based solely on the provided selected text
- **SC-004**: The chatbot widget appears on every page of the Docusaurus book and loads successfully 99% of the time
- **SC-005**: Embedding generation and indexing process completes successfully without errors for the entire textbook
- **SC-006**: The system properly handles missing embeddings by showing the correct error message
- **SC-007**: All previous chatbot files and code are removed, resulting in a clean codebase with no duplicate or unused files