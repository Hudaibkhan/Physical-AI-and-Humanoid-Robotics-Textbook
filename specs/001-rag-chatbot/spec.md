# Feature Specification: RAG Chatbot for Digital Book Website

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Title: Add RAG Chatbot to Digital Book Website (Using Gemini LLM)

Goal:
Integrate a Retrieval-Augmented Generation (RAG) chatbot into my digital book website. The chatbot should appear as a popup widget and answer user questions based only on my book's content or user-selected text. The main LLM must be Google Gemini, and embeddings should use any free embedding model (Gemini or OpenAI Mini).
/sp/i
Objectives:
1. Allow uploading the full book text, auto-chunking it, generating embeddings (using any free embedding model), and storing vectors in Qdrant Cloud Free Tier.
2. Build a FastAPI backend with:
   - /embed → create embeddings using Gemini or OpenAI Mini (free)
   - /search → query Qdrant for similar book chunks
   - /ask-agent → send question + selected text + retrieved chunks to the agent
3. Build a chatbot agent using OpenAI Agents SDK or ChatKit SDK that:
   - Uses Gemini as the main LLM for final answer generation
   - Uses backend RAG context
   - Answers strictly from book content
   - Says: "This information is not in the book" for irrelevant queries
4. Build a lightweight popup chatbot widget for the frontend:
   - Appears on every page of the book
   - Works with Vercel deployment
   - Supports user text highlight → chatbot answers from that highlighted text only
5. Deploy backend to Render.com Free Tier and frontend to Vercel free plan.
6. Fully connect:
   - Qdrant (vectors)
   - FastAPI (backend)
   - Gemini LLM
   - Embedding model
   - Chatbot widget on book site

Tech Stack:
- LLM: Gemini (flash, flash-lite, pro, etc.)
- Embeddings: Free model (Gemini embedding OR OpenAI Mini embedding)
- Backend: FastAPI (Python)
- Vector DB: Qdrant Cloud Free Tier
- Agent Runtime: OpenAI Agents SDK or ChatKit
- Hosting: Render (backend) + Vercel (frontend)
- Frontend: HTML + JS widget

Core Features:
- Highlight-to-answer (user selects text → chatbot uses only that text)
- RAG-powered global book search
- No hallucinations — strict book-only answers
- Popup chatbot with minimal UI and dark/light theme

Success Criteria:
- Book chunks uploaded to Qdrant successfully
- RAG pipeline responds accurately using Gemini LLM
- Popup chatbot loads on the website and communicates with backend
- System rejects out-of-book questions correctly

Deliverables:
- System architecture
- Project folder structure
- FastAPI backend code
- Qdrant upload script
- Popup widget HTML/JS
- Agent configuration (Gemini LLM + RAG)
- Deployment instructions (Render + Vercel)
- Integration guide for adding widget into book site"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Book Questions via Chatbot (Priority: P1)

As a reader of the digital book, I want to ask questions about the book content through a chatbot widget so that I can get accurate answers based on the book material without having to manually search through pages.

**Why this priority**: This is the core functionality that delivers the primary value of the feature - enabling readers to interact with the book content through natural language questions.

**Independent Test**: Can be fully tested by asking various questions about the book content and verifying that the chatbot provides accurate answers based only on the book material.

**Acceptance Scenarios**:

1. **Given** I am viewing a book page with the chatbot widget available, **When** I type a question related to the book content and submit it, **Then** I receive an accurate answer based on the book content.
2. **Given** I have asked a question that is not covered by the book content, **When** I submit the question, **Then** the chatbot responds with "This information is not in the book".

---

### User Story 2 - Highlight Text and Ask Questions (Priority: P2)

As a reader, I want to select/highlight specific text on the book page and ask questions about only that highlighted text, so that I can get focused answers on the specific content I'm interested in.

**Why this priority**: This enhances the user experience by allowing context-specific queries and enables more precise information retrieval.

**Independent Test**: Can be tested by highlighting text on a book page, asking a question related to that text, and verifying the response is based only on the highlighted content.

**Acceptance Scenarios**:

1. **Given** I have selected/highlighted text on a book page, **When** I ask a question about that highlighted text, **Then** the chatbot provides an answer based only on the highlighted text content.

---

### User Story 3 - Access Chatbot Across All Book Pages (Priority: P3)

As a reader, I want the chatbot widget to be available on every page of the book, so that I can ask questions regardless of which page I'm currently viewing.

**Why this priority**: This ensures consistent user experience across the entire book, making the feature accessible wherever the user might need it.

**Independent Test**: Can be tested by navigating to different pages of the book and verifying the chatbot widget is present and functional on each page.

**Acceptance Scenarios**:

1. **Given** I am viewing any page of the book, **When** I interact with the chatbot widget, **Then** the widget is functional and allows me to ask questions about the book content.

---

### Edge Cases

- What happens when the book content has not been fully indexed in the vector database yet?
- How does the system handle very long questions or questions with special characters?
- What happens when the chatbot cannot find relevant information in the book for a query?
- How does the system handle multiple simultaneous users asking questions?
- What happens if the vector database is temporarily unavailable?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a popup chatbot widget that appears on every page of the digital book website
- **FR-002**: System MUST allow users to ask questions about the book content through the chatbot interface
- **FR-003**: System MUST answer user questions based only on the book content using Retrieval-Augmented Generation (RAG)
- **FR-004**: System MUST respond with "This information is not in the book" when a question cannot be answered using book content
- **FR-005**: System MUST allow users to select/highlight text on the book page and ask questions specifically about that highlighted text
- **FR-006**: System MUST store book content as vector embeddings in a vector database (Qdrant Cloud Free Tier)
- **FR-007**: System MUST use Google Gemini as the primary LLM for answer generation
- **FR-008**: System MUST use a free embedding model (either Gemini or OpenAI Mini) for generating vector embeddings
- **FR-009**: System MUST provide both dark and light theme options for the chatbot UI
- **FR-010**: System MUST be deployable to Vercel for the frontend and Render.com for the backend

*Example of marking unclear requirements:*

- **FR-011**: System MUST handle book content uploads in Markdown format
- **FR-012**: System MUST support up to 50 concurrent users

### Key Entities

- **Book Content**: The digital book material that will be chunked and converted to vector embeddings for RAG retrieval
- **Vector Embeddings**: Numerical representations of book content chunks stored in Qdrant for semantic search
- **User Query**: Questions submitted by readers through the chatbot interface
- **Chat Response**: Answers generated by the Gemini LLM based on retrieved book content

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Book content is successfully uploaded and indexed in the vector database with 100% of content chunks properly embedded
- **SC-002**: RAG pipeline responds to user queries with accurate answers based on book content in under 5 seconds
- **SC-003**: The popup chatbot widget loads successfully on 100% of book pages without impacting page performance
- **SC-004**: The system correctly rejects out-of-book questions with "This information is not in the book" message 100% of the time
- **SC-005**: At least 90% of user questions about book content receive accurate, relevant answers based on the book material
- **SC-006**: The backend service successfully deploys to Render.com Free Tier and remains accessible
- **SC-007**: The frontend widget successfully deploys to Vercel and integrates seamlessly with the existing book website
