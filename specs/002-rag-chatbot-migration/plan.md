# Implementation Plan: RAG Chatbot Integration Fix + Agent SDK Migration

**Feature**: RAG Chatbot Integration Fix + Agent SDK Migration
**Branch**: 002-rag-chatbot-migration
**Created**: 2025-12-10
**Status**: Draft
**Author**: Claude

## Technical Context

This plan addresses the migration of the existing RAG chatbot to use the official OpenAI Agent SDK with Gemini as the LLM provider. The system will integrate with Qdrant for vector storage and Cohere for embeddings, while implementing a ChatKit frontend widget for Docusaurus.

### Architecture Overview

The system will consist of:
- **Backend**: FastAPI application with OpenAI Agent SDK integration
- **Vector DB**: Qdrant Cloud for storing document embeddings
- **Embeddings**: Cohere free embedding model
- **LLM**: Gemini API via OpenAI Agent SDK
- **Frontend**: ChatKit widget embedded in Docusaurus pages
- **Data Flow**: Text content → Cohere embeddings → Qdrant storage → RAG retrieval → Gemini response

### Technology Stack

- **Backend**: Python, FastAPI
- **AI/ML**: OpenAI Agent SDK, Gemini API, Cohere embeddings
- **Vector DB**: Qdrant Cloud
- **Frontend**: ChatKit SDK, JavaScript
- **Documentation**: Docusaurus v3
- **Deployment**: Railway (backend), GitHub Pages (frontend)

### Unknowns & Dependencies

All unknowns have been resolved through research documented in research.md:

- OpenAI Agent SDK with custom Gemini API integration approach defined
- MCP tools integration patterns for RAG actions documented
- ChatKit widget integration with OpenAI Agent detailed
- Qdrant collection schema for Cohere embeddings specified (1024 dimensions, cosine distance)

## Constitution Check

### Compliance Verification

- ✅ **Spec-Driven Writing**: Following `/sp.plan` after `/sp.specify`
- ✅ **Technical Reliability**: Using official SDKs and established patterns
- ✅ **Beginner-Friendly + Professional Tone**: Clear documentation and structure
- ✅ **Documentation Quality**: Following Docusaurus standards
- ✅ **Consistency**: Using established patterns across modules
- ✅ **Book Requirements**: Supporting textbook content interaction
- ✅ **Technical Constraints**: Compatible with Docusaurus and GitHub Pages

### Gate Evaluation

- **Architecture**: Follows microservices approach with clear separation of concerns
- **Technology**: Uses official SDKs and established libraries
- **Standards**: Complies with Docusaurus and accessibility requirements
- **Performance**: Designed with scalability in mind

## Phase 0: Research & Discovery

### Completed Research

Research has been completed and documented in [research.md](./research.md), addressing:

1. **OpenAI Agent SDK with Gemini Integration**
   - Custom LLM provider wrapper approach defined
   - Implementation pattern using Google AI SDK documented

2. **MCP Tools Integration**
   - RAG action tools specification completed
   - Tool definition patterns documented

3. **Qdrant with Cohere Embeddings**
   - Vector dimension confirmed as 1024 for Cohere embeddings
   - Collection schema with cosine distance specified

4. **ChatKit Widget Integration**
   - Custom agent integration approach documented
   - Docusaurus implementation pattern defined

### Outcomes Achieved

- ✅ Understanding of how to integrate Gemini with OpenAI Agent SDK
- ✅ Knowledge of MCP tools usage patterns for RAG actions
- ✅ Qdrant collection schema for Cohere embeddings (1024 dimensions, cosine distance)
- ✅ ChatKit integration approach with custom agents documented

## Phase 1: System Preparation

### 1.1 Clean Old Chatbot Files

**Objective**: Remove all previous chatbot implementations to avoid conflicts

**Tasks**:
- [ ] Identify and list all existing chatbot-related files
- [ ] Remove old JavaScript files (e.g., /chatbox.js)
- [ ] Remove unused CSS files (e.g., css/chatbot.css)
- [ ] Delete old agent helper scripts
- [ ] Remove unused API routes
- [ ] Verify no duplicate or conflicting code remains

**Files to Remove**:
- `/chatbox.js`
- `css/chatbot.css`
- Any files containing "old", "legacy", "backup" in names related to chatbot
- Any unused API route files

### 1.2 Environment Variables Setup

**Objective**: Configure required environment variables for the new system

**Variables Required**:
- `GEMINI_API_KEY`: API key for Gemini service
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `COHERE_API_KEY`: API key for Cohere embeddings
- `AGENT_ID`: ID for the created OpenAI Agent (after creation)

**Implementation**:
- [ ] Create `.env.example` file with all required variables
- [ ] Update deployment configuration for Railway
- [ ] Document environment setup process

### 1.3 Agent Context Update
**Objective**: Update agent context with new technologies and patterns from this plan

**Implementation**:
- [x] Created agent context file at `.specify/memory/agent-context-claude.md`
- [x] Added backend technologies: FastAPI, OpenAI Agent SDK, Google Generative AI SDK, Cohere API, Qdrant Client
- [x] Added frontend technologies: ChatKit SDK, JavaScript
- [x] Documented architecture patterns and implementation details
- [x] Preserved for future agent reference

## Phase 2: Embedding Pipeline (Cohere)

### 2.1 Embed Text Utility

**Objective**: Create utility function for generating embeddings with Cohere

**Implementation**:
- [ ] Implement `embed_text(text)` function using Cohere API
- [ ] Handle API rate limits and errors
- [ ] Cache embeddings to avoid repeated API calls
- [ ] Validate embedding dimensions (expected 1024 for Cohere)

### 2.2 Embedding Pipeline Script

**Objective**: Build complete pipeline for processing textbook content

**Components**:
- [ ] Load markdown files from textbook content
- [ ] Chunk text into appropriate sizes (e.g., 512 tokens)
- [ ] Generate embeddings for each chunk
- [ ] Upsert embeddings to Qdrant "book_chunks" collection
- [ ] Add metadata for each chunk (source file, position, etc.)

**Implementation Steps**:
- [ ] Create `embed_pipeline.py` script
- [ ] Implement markdown file loader
- [ ] Implement text chunker with overlap handling
- [ ] Implement embedding generation with Cohere
- [ ] Implement Qdrant upsert functionality

### 2.3 Verification System

**Objective**: Ensure embeddings exist before RAG queries

**Implementation**:
- [ ] Create function to check if embeddings exist in Qdrant
- [ ] Implement verification step before RAG queries
- [ ] Return "Embedding not found: Re-run embed pipeline" if missing

## Phase 3: Qdrant Vector Database Setup

### 3.1 Collection Creation

**Objective**: Set up Qdrant collection for storing book content embeddings

**Configuration**:
- Collection name: "book_chunks"
- Vector size: 1024 (Cohere embedding dimension)
- Distance function: cosine
- Additional payload: source file, chunk position, metadata

**Implementation**:
- [ ] Create Qdrant client connection
- [ ] Implement collection creation with proper schema
- [ ] Add error handling for collection creation

### 3.2 FastAPI Services

**Objective**: Implement required RAG endpoints

**Endpoints to Implement**:
- `POST /embed`: Create embeddings and upsert to Qdrant
- `POST /rag`: Retrieve chunks and generate response
- `POST /rag/selected`: Selected-text-only RAG

**Implementation**:
- [ ] Create FastAPI application structure
- [ ] Implement `/embed` endpoint
- [ ] Implement `/rag` endpoint
- [ ] Implement `/rag/selected` endpoint
- [ ] Add Qdrant client integration
- [ ] Add retrieval logic for RAG agent tool

## Phase 4: OpenAI Agent SDK + Gemini Integration

### 4.1 RAG Agent Creation

**Objective**: Create RAG agent using OpenAI Agent SDK with Gemini

**Implementation**:
- [ ] Create OpenAI Agent using official SDK
- [ ] Add `retrieve_chunks(query)` tool for Qdrant retrieval
- [ ] Add `answer_with_context(question, chunks)` tool
- [ ] Configure agent to use Gemini as LLM provider

### 4.2 MCP Tools Integration

**Objective**: Integrate MCP tools for RAG actions

**Implementation**:
- [ ] Research MCP tools from context7 guidelines
- [ ] Implement required MCP tools for RAG functionality
- [ ] Test agent response through API

## Phase 5: FastAPI Backend (Core API Layer)

### 5.1 API Endpoints Implementation

**Objective**: Implement core API layer for the application

**Endpoints**:
- `/api/rag`: Qdrant RAG + agent answer
- `/api/rag/selected`: Selected-text-only pipeline
- `/api/conversation`: ChatKit agent call

**Implementation**:
- [ ] Implement `/api/rag` endpoint
- [ ] Implement `/api/rag/selected` endpoint
- [ ] Implement `/api/conversation` endpoint
- [ ] Add proper error handling
- [ ] Add fallback message when agent is unreachable

### 5.2 Error Handling

**Objective**: Ensure robust error handling throughout the system

**Implementation**:
- [ ] Add try-catch blocks for all external API calls
- [ ] Implement fallback responses when agent is unreachable
- [ ] Add logging for debugging purposes
- [ ] Create user-friendly error messages

## Phase 6: ChatKit Frontend Widget

### 6.1 Widget Integration

**Objective**: Integrate ChatKit widget into Docusaurus pages

**Implementation**:
- [ ] Add ChatKit script to Docusaurus template
- [ ] Create floating widget in bottom-right corner
- [ ] Connect widget to agent ID
- [ ] Add ARIA labels for accessibility
- [ ] Implement fallback display for unavailable chatbot

### 6.2 Widget Features

**Implementation**:
- [ ] Ensure widget appears on every Docusaurus page
- [ ] Add "Open chatbot" ARIA label
- [ ] Implement fallback message: "Chatbot unavailable. Try again later."
- [ ] Test widget functionality across different pages

## Phase 7: Selected-Text RAG UI

### 7.1 Text Selection Listener

**Objective**: Implement functionality to handle selected text queries

**Implementation**:
- [ ] Add text selection listener to book pages
- [ ] Create "Ask from Highlighted Text" functionality
- [ ] Send selected text to `/rag/selected` endpoint
- [ ] Display agent answer within the widget

## Phase 8: Final Testing

### 8.1 System Testing

**Objective**: Validate all components work together correctly

**Tests**:
- [ ] Test embedding pipeline functionality
- [ ] Test Qdrant retrieval accuracy
- [ ] Test agent response quality
- [ ] Test widget visibility on all Docusaurus pages
- [ ] Fix console errors and CORS issues
- [ ] Verify all old files have been removed

## Phase 9: Deployment

### 9.1 Production Deployment

**Objective**: Deploy the complete system to production

**Tasks**:
- [ ] Deploy backend to Railway
- [ ] Deploy book to GitHub Pages
- [ ] Verify widget functionality in production
- [ ] Test end-to-end functionality

## Data Model

### Entities

1. **BookContentChunk**
   - id: string (Qdrant point ID)
   - content: string (text chunk)
   - embedding: float[] (Cohere embedding vector)
   - source_file: string (markdown file path)
   - position: number (position in original document)
   - metadata: object (additional information)

2. **ChatSession**
   - id: string (session identifier)
   - user_id: string (user identifier)
   - messages: array (conversation history)
   - created_at: datetime
   - updated_at: datetime

3. **AgentResponse**
   - id: string (response identifier)
   - session_id: string (associated session)
   - query: string (original user query)
   - context_chunks: array (retrieved chunks)
   - response: string (generated answer)
   - timestamp: datetime

## API Contracts

### Embedding Service
```
POST /embed
Request: { "source_path": "path/to/markdown/files" }
Response: { "status": "success", "chunks_processed": number }
```

### RAG Service
```
POST /rag
Request: { "query": "user question", "context": "optional context" }
Response: { "answer": "generated response", "sources": ["chunk_ids"] }
```

### Selected Text RAG Service
```
POST /rag/selected
Request: { "selected_text": "user selected text", "query": "question about selection" }
Response: { "answer": "response based on selected text" }
```

### Conversation Service
```
POST /api/conversation
Request: { "message": "user message", "session_id": "optional" }
Response: { "response": "agent response", "session_id": "session identifier" }
```

## Quickstart Guide

1. **Setup Environment**:
   ```bash
   # Install dependencies
   pip install fastapi uvicorn openai cohere qdrant-client python-dotenv

   # Copy environment variables
   cp .env.example .env
   # Add your API keys to .env
   ```

2. **Run Embedding Pipeline**:
   ```bash
   python embed_pipeline.py
   ```

3. **Start Backend Server**:
   ```bash
   uvicorn main:app --reload
   ```

4. **Integrate Frontend**:
   - Add ChatKit script to Docusaurus
   - Configure agent connection

## Risk Analysis

### High-Risk Areas

1. **API Key Management**: Secure handling of multiple API keys
2. **Qdrant Connection**: Network reliability and rate limits
3. **Agent Integration**: Compatibility between OpenAI Agent SDK and Gemini
4. **CORS Issues**: Cross-origin requests between frontend and backend

### Mitigation Strategies

1. **API Keys**: Use environment variables and proper secret management
2. **Qdrant**: Implement retry logic and connection pooling
3. **Agent**: Thorough testing with fallback mechanisms
4. **CORS**: Proper configuration for development and production

## Success Criteria

- [ ] Agent successfully responds to queries using RAG
- [ ] Embedding pipeline processes all textbook content
- [ ] ChatKit widget appears on all Docusaurus pages
- [ ] Selected text RAG bypasses Qdrant as required
- [ ] All old chatbot files removed
- [ ] No console errors during initialization
- [ ] System handles missing embeddings gracefully