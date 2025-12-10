---
id: 1
title: "rag-chatbot-migration-spec"
stage: "spec"
date_iso: "2025-12-10"
surface: "agent"
model: "gemini-2.5-flash"
feature: "rag-chatbot-migration"
branch: "002-rag-chatbot-migration"
user: "user"
command: "/sp.specify"
labels: ["specification", "rag-chatbot", "migration"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml: |
  - specs/002-rag-chatbot-migration/spec.md
  - specs/002-rag-chatbot-migration/checklists/requirements.md
tests_yaml: |
  - "spec creation"
prompt_text: |
  "RAG Chatbot Integration Fix + Agent SDK Migration"

  REQ-001: Agent Creation Using OpenAI Agent SDK
    - The RAG chatbot agent MUST be implemented using the official OpenAI Agent SDK.
    - Agent MUST expose a function/tool for:
        - retrieve_chunks(query) → fetch from Qdrant
        - answer_with_context(question, chunks)
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
    - No duplicate or unused files
response_text: |
  Created specification for RAG Chatbot Integration Fix + Agent SDK Migration feature in specs/002-rag-chatbot-migration/spec.md with user scenarios, functional requirements, and success criteria. Created quality checklist at specs/002-rag-chatbot-migration/checklists/requirements.md. Created new branch 002-rag-chatbot-migration. The specification includes requirements for OpenAI Agent SDK implementation, Qdrant integration, ChatKit frontend, embeddings verification, and file cleanup as specified in the original requirements.
outcome: "specification created successfully"
evaluation: "spec meets all requirements and is ready for planning phase"
---
