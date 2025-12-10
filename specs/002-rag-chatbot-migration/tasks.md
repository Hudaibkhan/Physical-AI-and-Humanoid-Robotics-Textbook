# Tasks: RAG Chatbot Integration Fix + Agent SDK Migration

**Feature**: RAG Chatbot Integration Fix + Agent SDK Migration
**Branch**: 002-rag-chatbot-migration
**Created**: 2025-12-10
**Status**: Draft
**Author**: Claude

## Phase 1: Setup (Project Initialization)

### Goal
Initialize the project structure with required dependencies and configuration files.

### Tasks
- [X] T001 Create backend directory structure: `backend/`, `backend/src/`, `backend/src/api/`, `backend/src/models/`, `backend/src/services/`, `backend/src/tools/`, `backend/src/utils/`
- [X] T002 Create frontend directory structure: `frontend/`, `frontend/src/`, `frontend/src/components/`
- [X] T003 Initialize Python virtual environment and requirements.txt with: fastapi, uvicorn, python-dotenv, openai, cohere, qdrant-client, google-generativeai, pydantic
- [X] T004 Create .env.example file with required environment variables: GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY
- [X] T005 Create main FastAPI application file: `backend/src/main.py`
- [X] T006 Set up basic FastAPI configuration with CORS middleware

## Phase 2: Foundational (Blocking Prerequisites)

### Goal
Implement foundational components that all user stories depend on.

### Tasks
- [X] T007 [P] Implement Qdrant client connection utility in `backend/src/utils/qdrant_client.py`
- [X] T008 [P] Create Qdrant collection "book_chunks" with 1024 dimensions and cosine distance
- [X] T009 [P] Implement Cohere embedding utility in `backend/src/utils/embedding_utils.py`
- [X] T010 [P] Create custom LLM provider wrapper for Gemini integration in `backend/src/utils/gemini_provider.py`
- [X] T011 [P] Implement error handling utilities in `backend/src/utils/error_handlers.py`
- [X] T012 Create agent tools base classes in `backend/src/tools/base.py`
- [X] T013 [P] Implement environment variable validation in `backend/src/config.py`

## Phase 3: User Story 1 - Basic Chatbot Interaction (Priority: P1)

### Goal
Implement core RAG chatbot functionality that allows users to ask questions and receive answers based on textbook content.

### Independent Test Criteria
Can be fully tested by asking the chatbot a question and verifying it provides a relevant response based on the book content, delivering the primary value proposition of the feature.

### Tasks
- [X] T014 [P] [US1] Create retrieve_chunks tool in `backend/src/tools/retrieve_chunks.py` that queries Qdrant
- [X] T015 [P] [US1] Create answer_with_context tool in `backend/src/tools/answer_with_context.py` that generates responses
- [X] T016 [US1] Implement OpenAI Agent with custom tools in `backend/src/agents/rag_agent.py`
- [X] T017 [US1] Create /rag endpoint in `backend/src/api/rag_routes.py` for RAG queries
- [X] T018 [US1] Implement embedding verification logic to check if embeddings exist before queries
- [X] T019 [US1] Add "Embedding not found: Re-run embed pipeline" response when embeddings are missing
- [X] T020 [US1] Implement basic chat session management in `backend/src/models/chat_session.py`
- [X] T021 [US2] Create basic ChatKit integration in `frontend/src/components/chat-widget.js`
- [X] T022 [US1] Test basic chat functionality with sample queries

## Phase 4: User Story 2 - Selected Text RAG Mode (Priority: P2)

### Goal
Implement functionality to allow users to select specific text and ask questions about just that text, bypassing Qdrant.

### Independent Test Criteria
Can be tested by selecting text in the book, asking a question about it, and verifying the chatbot responds based only on the selected text without referencing other parts of the book.

### Tasks
- [X] T023 [P] [US2] Create /rag/selected endpoint in `backend/src/api/rag_routes.py` for selected-text-only queries
- [X] T024 [US2] Implement frontend text selection listener in `frontend/src/components/text-selector.js`
- [X] T025 [US2] Add "Ask from Highlighted Text" functionality to send selected text to `/rag/selected`
- [X] T026 [US2] Ensure selected text RAG bypasses Qdrant and answers only from provided text
- [X] T027 [US2] Integrate selected text functionality with ChatKit widget
- [X] T028 [US2] Test selected text RAG mode with various text selections

## Phase 5: User Story 3 - Embeddings and Content Indexing (Priority: P3)

### Goal
Implement the embedding pipeline to generate embeddings for textbook content and index it in Qdrant.

### Independent Test Criteria
Can be tested by running the embed pipeline and verifying that content is properly indexed in Qdrant, delivering the indexing capability as a standalone feature.

### Tasks
- [X] T029 [P] [US3] Create embed_text utility function in `backend/src/utils/embedding_utils.py`
- [X] T030 [P] [US3] Implement markdown file loader in `backend/src/utils/document_loader.py`
- [X] T031 [P] [US3] Create text chunker utility in `backend/src/utils/text_chunker.py`
- [X] T032 [US3] Implement embedding pipeline script in `backend/src/scripts/embed_pipeline.py`
- [X] T033 [US3] Create /embed endpoint in `backend/src/api/embed_routes.py` for processing documents
- [X] T034 [US3] Implement Qdrant upsert functionality for embeddings
- [X] T035 [US3] Add metadata handling for book chunks (source file, position, etc.)
- [X] T036 [US3] Test embedding pipeline with sample textbook content

## Phase 6: Frontend Widget Implementation

### Goal
Implement the ChatKit widget with proper positioning, accessibility, and fallback functionality.

### Tasks
- [X] T037 [P] Add ChatKit script to Docusaurus template in `docusaurus.config.js`
- [X] T038 [P] Create floating widget positioning in bottom-right corner
- [X] T039 [P] Add ARIA label "Open chatbot" to widget button for accessibility
- [X] T040 Implement fallback display "Chatbot unavailable. Try again later." when backend fails
- [X] T041 Ensure widget appears on every Docusaurus page
- [X] T042 Connect widget to the created agent ID
- [X] T043 Test widget functionality across different Docusaurus pages

## Phase 7: System Preparation & Cleanup

### Goal
Clean old chatbot files and prepare the system for the new implementation.

### Tasks
- [X] T044 [P] Identify and list all existing chatbot-related files for removal
- [X] T045 [P] Remove old JavaScript files (e.g., /chatbox.js)
- [X] T046 [P] Remove unused CSS files (e.g., css/chatbot.css)
- [X] T047 [P] Delete old agent helper scripts
- [X] T048 [P] Remove unused API routes not used by the new agent system
- [X] T049 [P] Verify no duplicate or conflicting code remains after cleanup

## Phase 8: Integration & Testing

### Goal
Integrate all components and perform comprehensive testing.

### Tasks
- [X] T050 [P] Integrate backend API with frontend ChatKit widget
- [X] T051 [P] Test end-to-end RAG functionality with real textbook content
- [X] T052 Test Qdrant retrieval accuracy
- [X] T053 Test agent response quality and relevance
- [X] T054 Test widget visibility on all Docusaurus pages
- [X] T055 Fix console errors and CORS issues
- [X] T056 Verify all old files have been removed
- [X] T057 Test error handling for missing embeddings
- [X] T058 Test fallback messages when services are unavailable

## Phase 9: Polish & Cross-Cutting Concerns

### Goal
Address final implementation details, documentation, and deployment preparation.

### Tasks
- [X] T059 Add comprehensive logging throughout the application
- [X] T060 Implement proper error boundaries and user-friendly error messages
- [X] T061 Add input validation and sanitization for all user inputs
- [X] T062 Optimize chunk sizes and embedding parameters based on content
- [X] T063 Update documentation with deployment instructions
- [X] T064 Create deployment configuration for Railway backend
- [X] T065 Test production deployment setup
- [X] T066 Final testing of complete system functionality

## Dependencies

### User Story Completion Order
1. **User Story 3 (Embeddings)** must be completed before User Story 1 (Basic Chatbot) can function properly
2. **User Story 1 (Basic Chatbot)** provides foundation for User Story 2 (Selected Text RAG)
3. **User Story 2 (Selected Text RAG)** can be developed in parallel with User Story 1 after core infrastructure exists

### Critical Path
- T001-T013 (Setup and Foundational) → T029-T036 (Embeddings) → T014-T022 (Basic Chatbot) → T023-T028 (Selected Text RAG)

## Parallel Execution Examples

### Per User Story 1 (Basic Chatbot):
- T014, T015 (Tools implementation) can run in parallel
- T017, T018 (Endpoints and verification) can run in parallel
- T021 (Frontend widget) can be developed in parallel with backend work

### Per User Story 2 (Selected Text RAG):
- T023 (Endpoint) and T024 (Frontend listener) can run in parallel
- T025, T026 (Integration tasks) can run in parallel

### Per User Story 3 (Embeddings):
- T029-T031 (Utilities) can run in parallel
- T033, T034 (Endpoint and upsert) can run in parallel

## Implementation Strategy

### MVP Scope (User Story 1 Only)
1. Complete Phase 1 (Setup) and Phase 2 (Foundational)
2. Complete Phase 3 (User Story 1 - Basic Chatbot)
3. Basic RAG functionality with Qdrant and Cohere embeddings
4. Simple ChatKit widget integration
5. Minimum viable functionality for users to ask questions and receive answers

### Incremental Delivery
1. MVP: Basic chat functionality (User Story 1)
2. Enhancement: Selected text RAG (User Story 2)
3. Foundation: Embedding pipeline (User Story 3)
4. Polish: Frontend widget and cleanup (Phases 6-9)