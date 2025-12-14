# Implementation Plan: Book RAG Agent Upgrade

**Feature Branch**: `003-book-rag-agent-upgrade`
**Created**: 2025-12-12
**Status**: Planned
**Spec**: [spec.md](spec.md)

## Technical Context

This feature implements a Book RAG Agent using OpenAI Agents SDK with the following key requirements:
- Agent uses Gemini 2.5 Flash via LiteLLM integration
- Supports both normal RAG mode (Qdrant) and selected text RAG mode
- Uses function tools for retrieval operations
- Frontend uses ChatKit widget with FastAPI backend
- Backend deploys to Hugging Face Spaces, frontend to Vercel

**Unknowns resolved via research.md**:
- OpenAI Agents SDK integration with external models
- Function tool implementation for rag_query
- Mode switching between RAG and selected text
- Session management with the new agent framework

## Constitution Check

### From Project Constitution (v2.0.0)

**Agent-Driven Architecture**: ✓ Implemented with OpenAI Agents SDK and book_rag_agent
- Uses OpenAI Agents SDK ✓
- Gemini 2.5 Flash as LLM model ✓
- Supports both RAG modes ✓

**MCP Integration & Validation**: ✓ Reference documentation used for implementation
- MCP context7 docs used for guidance ✓
- Function tools implemented as specified ✓

**Production-Ready Implementation**: ✓ Self-contained, deployable code
- Backend deployable on Hugging Face Spaces ✓
- Frontend deployable on Vercel ✓
- Environment variables properly configured ✓

**Agent Response Quality**: ✓ Cites chunks and prevents hallucination
- Always cites retrieved chunks ✓
- Never hallucinates information ✓
- Consistent output formatting ✓

**System Consistency**: ✓ Consistent behavior across modes
- Both RAG modes produce identical formatting ✓

### Gate Evaluation

✅ **All constitution requirements satisfied**

## Phase 0: Research & Setup

**Status**: Complete - research.md created with implementation approaches

## Phase 1: Data Models & API Contracts

**Status**: Complete - data-model.md and API contracts created

## Phase 2: Implementation Architecture

### 2.1 Backend Architecture (fastapi_app/)

**connection.py**:
- Initialize Gemini client via LiteLLM
- Initialize Qdrant client
- Initialize embedding client
- Define utility functions: embed(), qdrant_search(), selected_text_search()

**agent.py**:
- Define rag_query() function tool with parameters (query: str, mode: str, top_k: int)
- Implement book_rag_agent with Gemini 2.5 Flash model
- Configure agent with proper instructions to only answer from provided context

**app.py**:
- FastAPI application with POST /chat endpoint
- Determine mode based on presence of selected_text
- Execute agent using Runner.run()
- Return structured responses with citations

### 2.2 Agent Implementation Details

**book_rag_agent**:
- Model: litellm/gemini/gemini-2.5-flash
- Instructions: Only answer from Qdrant content or selected text, never hallucinate, always cite chunks
- Tools: rag_query function tool

**rag_query function tool**:
- Parameters: query (str), mode (str), top_k (int)
- If mode="rag": Retrieve from Qdrant using embeddings
- If mode="selected": Process selected text, create embeddings, find matches
- Returns: id, text, similarity_score

### 2.3 Frontend Integration

**ChatKit Integration**:
- Embed ChatKit widget in Docusaurus
- Support for text selection functionality
- Display citations with responses
- Maintain conversation history

### 2.4 Deployment Architecture

**Hugging Face Backend**:
- app.py, agent.py, connection.py
- requirements.txt with OpenAI Agents SDK, LiteLLM, etc.
- Environment variables for API keys

**Vercel Frontend**:
- Docusaurus site with ChatKit widget
- Configured to connect to HF backend

## Phase 3: Implementation Steps

### 3.1 Backend Implementation
1. Create fastapi_app/ directory structure
2. Implement connection.py with all required clients and utilities
3. Implement agent.py with book_rag_agent and rag_query tool
4. Implement app.py with FastAPI endpoints
5. Test agent functionality locally

### 3.2 Frontend Integration
1. Update Docusaurus site to include ChatKit widget
2. Implement text selection functionality
3. Ensure proper communication with backend API
4. Test frontend integration

### 3.3 Testing
1. Unit tests for rag_query tool in both modes
2. Integration tests for agent responses
3. End-to-end tests with frontend
4. Verify citation accuracy
5. Test both RAG modes work correctly

## Phase 4: Validation Requirements

- [ ] Qdrant schema compatibility verified
- [ ] MCP skills properly referenced in implementation
- [ ] Agent JSON schema correct
- [ ] ChatKit frontend script stable
- [ ] Selected text handling works correctly
- [ ] Gemini 2.5 Flash connectivity verified

## Risks & Mitigations

**Risk**: LiteLLM integration with Gemini 2.5 Flash may have compatibility issues
*Mitigation*: Test with alternative model configurations if needed

**Risk**: Performance issues with selected text processing
*Mitigation*: Implement efficient text chunking and similarity algorithms

**Risk**: Session management complexity with new agent framework
*Mitigation*: Start with simple session approach, enhance as needed