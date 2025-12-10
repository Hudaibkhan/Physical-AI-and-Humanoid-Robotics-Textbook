---
id: 2
title: "rag-chatbot-migration-plan"
stage: "plan"
date_iso: "2025-12-10"
surface: "agent"
model: "gemini-2.5-flash"
feature: "rag-chatbot-migration"
branch: "002-rag-chatbot-migration"
user: "user"
command: "/sp.plan"
labels: ["planning", "architecture", "rag-chatbot", "migration"]
links:
  spec: "../specs/002-rag-chatbot-migration/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml: |
  - specs/002-rag-chatbot-migration/plan.md
  - specs/002-rag-chatbot-migration/research.md
  - specs/002-rag-chatbot-migration/data-model.md
  - specs/002-rag-chatbot-migration/contracts/openapi.yaml
  - specs/002-rag-chatbot-migration/quickstart.md
  - .specify/memory/agent-context-claude.md
tests_yaml: |
  - "plan completeness check"
  - "research validation"
  - "data model validation"
  - "api contract validation"
prompt_text: |

  PHASE 1 — System Preparation
    1. Clean old chatbot files
        - Remove previous JS/CSS widget attempts
        - Delete unused API code and old agent scripts
    2. Prepare environment variables
        - GEMINI_API_KEY
        - QDRANT_URL
        - QDRANT_API_KEY
        - COHERE_API_KEY
        - AGENT_ID (from OpenAI Agent SDK after creation)

  PHASE 2 — Embedding Pipeline (Cohere)
    1. Implement an embed_text() utility using Cohere free embedding model
    2. Build an embedding pipeline script:
          - Load markdown files
          - Chunk text
          - Generate embeddings
          - Upsert into Qdrant ("book_chunks" collection)
    3. Add verification step:
          - If embeddings missing → print "Embedding not found: run embed pipeline"

  PHASE 3 — Qdrant Vector Database Setup
    1. Create collection "book_chunks" via API
        - size: 1024 (Cohere embedding dimension)
        - distance: cosine
    2. Implement FastAPI services:
        - POST /embed
        - POST /rag
        - POST /rag/selected
    3. Connect Qdrant client in backend
    4. Add retrieval logic for RAG agent tool

  PHASE 4 — OpenAI Agent SDK + Gemini Integration
    1. Create RAG agent via OpenAI Agent SDK
        - add tool: retrieve_chunks()
        - add tool: answer_with_context()
    2. Configure agent to use Gemini as the LLM provider
    3. Add MCP tools for RAG actions (from context7 guidelines)
    4. Test: agent should respond through API

  PHASE 5 — FastAPI Backend (Core API Layer)
    1. Implement endpoints:
          - /api/rag → Qdrant RAG + agent answer
          - /api/rag/selected → Selected-text-only pipeline
          - /api/conversation → ChatKit agent call
    2. Add error handling:
          - If agent unreachable → return fallback message
    3. Deploy backend on Railway

  PHASE 6 — ChatKit Frontend Widget
    1. Add ChatKit `<script>` to Docusaurus
    2. Create floating widget (bottom-right)
    3. Connect widget to AGENT_ID
    4. Add ARIA labels for accessibility
    5. Add fallback display:
          - "Chatbot unavailable. Try again later."

  PHASE 7 — Selected-Text RAG UI
    1. Add text selection listener in book pages
    2. On "Ask from Highlighted Text" → send `/rag/selected` request
    3. Display agent answer inside the widget

  PHASE 8 — Final Testing
    1. Test embedding pipeline
    2. Test Qdrant retrieval
    3. Test agent responses
    4. Test widget visibility on every Docusaurus page
    5. Fix console errors & CORS issues
    6. Ensure all old files removed

  PHASE 9 — Deployment
    1. Deploy backend → Railway
    2. Deploy book → GitHub Pages or Vercel
    3. Verify: widget fully functional in production
response_text: |
  Created comprehensive implementation plan for RAG Chatbot Integration Fix + Agent SDK Migration feature. The plan includes:

  1. Complete implementation plan (plan.md) with 9 phases covering all requirements
  2. Research document (research.md) resolving all technical unknowns:
     - OpenAI Agent SDK with Gemini API integration approach
     - MCP tools integration for RAG actions
     - ChatKit widget integration with custom agent
     - Qdrant schema for Cohere embeddings (1024 dimensions, cosine distance)
  3. Data model (data-model.md) defining all entities and relationships
  4. API contracts (contracts/openapi.yaml) with complete OpenAPI specification
  5. Quickstart guide (quickstart.md) for setup and deployment
  6. Agent context file (.specify/memory/agent-context-claude.md) with relevant technologies

  All unknowns from the technical context have been resolved through research. The plan follows the constitution requirements and is ready for the tasks phase.
outcome: "implementation plan completed successfully"
evaluation: "plan addresses all requirements and is ready for task breakdown"
---
