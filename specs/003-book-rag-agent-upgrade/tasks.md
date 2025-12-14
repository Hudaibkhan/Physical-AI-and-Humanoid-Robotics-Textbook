# Implementation Tasks: Book RAG Agent Upgrade

**Feature Branch**: `003-book-rag-agent-upgrade`
**Created**: 2025-12-12
**Status**: Planned
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)

## Implementation Strategy

**MVP Scope**: User Story 1 (Basic Book Query) provides core functionality with normal RAG mode using Qdrant and Gemini 2.5 Flash. This delivers immediate value as a working RAG system.

**Incremental Delivery**:
- Phase 1-2: Foundation (project setup, core dependencies)
- Phase 3: User Story 1 (P1) - Basic RAG functionality
- Phase 4: User Story 2 (P2) - Selected text functionality
- Phase 5: User Story 3 (P3) - Session management
- Phase 6: Polish & deployment

## Dependencies

- User Story 1 (P1) - Core functionality, no dependencies
- User Story 2 (P2) - Depends on US1 foundation
- User Story 3 (P3) - Depends on US1 foundation
- Final Phase - Depends on all user stories

## Parallel Execution Examples

**User Story 1 (P1)**:
- T005-T007 [P] - Connection utilities can be developed in parallel
- T008-T009 [P] - Agent components can be developed in parallel with connection layer

**User Story 2 (P2)**:
- Selected text processing can be developed in parallel with US1 once foundation is complete

## Phase 1: Setup

**Goal**: Initialize project structure and install dependencies for OpenAI Agents SDK with LiteLLM integration

- [x] T001 Create fastapi_app/ directory structure
- [x] T002 Install OpenAI Agents SDK, LiteLLM, and related dependencies in requirements.txt
- [x] T003 Set up environment variables for GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY
- [x] T004 Create basic FastAPI app skeleton in app.py

## Phase 2: Foundational Components

**Goal**: Implement core connection layer with Qdrant, Gemini, and embedding utilities

- [x] T005 Implement Qdrant client initialization in connection.py
- [x] T006 Implement Gemini client via LiteLLM initialization in connection.py
- [x] T007 Implement embedding utility functions in connection.py
- [x] T008 Implement qdrant_search function in connection.py
- [x] T009 Implement selected_text_search function in connection.py
- [x] T010 Create rag_query function tool skeleton in agent.py

## Phase 3: User Story 1 - Basic Book Query with OpenAI Agents (Priority: P1)

**Goal**: Implement core RAG functionality with Qdrant and Gemini 2.5 Flash

**Independent Test Criteria**: Can ask questions about book content and receive accurate answers from Qdrant with citations

- [x] T011 [US1] Implement rag_query function tool with "rag" mode for Qdrant retrieval in agent.py
- [x] T012 [US1] Implement book_rag_agent with Gemini 2.5 Flash model in agent.py
- [x] T013 [US1] Configure agent instructions to only answer from Qdrant content and cite chunks in agent.py
- [x] T014 [US1] Implement POST /chat endpoint in app.py to handle normal RAG mode
- [x] T015 [US1] Add logic to determine mode based on selected_text presence in app.py
- [x] T016 [US1] Implement agent execution using Runner.run() in app.py
- [x] T017 [US1] Format response with citations and source chunks in app.py
- [x] T018 [US1] Test basic Qdrant retrieval functionality
- [x] T019 [US1] Verify agent responses include proper citations
- [x] T020 [US1] Verify agent does not hallucinate information

## Phase 4: User Story 2 - Selected Text Query with OpenAI Agents (Priority: P2)

**Goal**: Implement selected text RAG mode where agent answers only from user-provided text

**Independent Test Criteria**: Can provide selected text and ask questions, with agent responding based only on selected text

- [x] T021 [US2] Enhance rag_query function tool to handle "selected" mode for text processing in agent.py
- [x] T022 [US2] Implement selected text chunking and embedding logic in connection.py
- [x] T023 [US2] Update POST /chat endpoint to handle selected text mode in app.py
- [x] T024 [US2] Test selected text mode functionality
- [x] T025 [US2] Verify agent only uses selected text (not Qdrant) in selected mode
- [x] T026 [US2] Ensure consistent response formatting between both modes

## Phase 5: User Story 3 - Chat History and Session Management (Priority: P3)

**Goal**: Implement conversation context maintenance across multiple questions in a session

**Independent Test Criteria**: Can ask follow-up questions in same session and agent maintains conversation context

- [x] T027 [US3] Implement session management utilities in connection.py
- [x] T028 [US3] Update agent to support conversation context in app.py
- [x] T029 [US3] Add session creation and management to POST /chat endpoint in app.py
- [x] T030 [US3] Test session continuity with follow-up questions
- [x] T031 [US3] Verify conversation history is properly maintained

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Complete implementation with error handling, validation, and deployment configuration

- [x] T032 Add error handling for Qdrant unavailability
- [x] T033 Implement validation for very long selected text
- [x] T034 Add fallback messages when query cannot be answered from context
- [x] T035 Create requirements.txt with all necessary dependencies
- [ ] T036 Update Docusaurus site to include ChatKit widget
- [ ] T037 Test frontend integration with backend API
- [x] T038 Verify all validation requirements from constitution are met
- [x] T039 Test deployment configuration for Hugging Face Spaces
- [x] T040 Document the implementation for deployment to Vercel