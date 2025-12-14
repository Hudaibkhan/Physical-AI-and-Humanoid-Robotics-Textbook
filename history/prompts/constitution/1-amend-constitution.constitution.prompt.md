---
id: 1
title: "amend-constitution"
stage: constitution
date: "2025-12-12"
surface: "agent"
model: "gemini-2.5-flash"
feature: "none"
branch: "main"
user: "user"
command: "/sp.constitution"
labels: ["constitution", "update", "project-structure"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Project Constitution Update

## Summary
Updated the project constitution to reflect the new architecture using Spec-Kit Plus, Claude Code, MCP tools, OpenAI Agents SDK, ChatKit UI, FastAPI backend, and Qdrant vector database.

## Changes
- Major version update from 1.1.0 to 2.0.0 due to significant architectural changes
- Replaced book creation principles with AI-powered RAG chatbot principles
- Added detailed sections for Agent Architecture, MCP Integration, Vector Database, Chat Frontend, and Backend Architecture
- Defined clear output rules, quality rules, and validation requirements
- Maintained governance and success criteria aligned with new architecture

## Prompt Text
```
# Project Constitution (Updated with MCP + context7)

This project will use Spec-Kit Plus, Claude Code, MCP tools, OpenAI Agents SDK, ChatKit UI, FastAPI backend, and Qdrant vector database.

## Core Rules

1. Agent Architecture
   - All chatbot intelligence must run through **OpenAI Agents SDK**.
   - The LLM model used by the agent must always be **Gemini 2.5 Flash**.
   - The agent must use the `Responses` API format for generating answers.
   - Agent must support two modes:
       a. Normal RAG mode (retrieve relevant chunks from Qdrant)
       b. Selected Text mode (answer ONLY from selected text supplied by the user, similar output formatting as normal RAG mode)
   - Agent implementation details must follow **MCP context7 guidance**:
       - context7 will provide documentation and instructions on:
           - how to create OpenAI Agents SDK agents
           - how to connect them to external LLMs (Gemini 2.5 Flash)
           - how to implement tool calls for Qdrant vector retrieval
           - how to handle selected text as context

2. MCP Integration
   - MCP tools must be used as skills for:
       - Chunking markdown
       - Creating embeddings
       - Qdrant upsert/search
       - Document selection & filtering
   - The agent must call these MCP tools via OpenAI Agents SDK tools API.
   - Use **MCP context7** as reference for all skill implementations to ensure standardization and reuse.

3. Vector Database (Qdrant)
   - Qdrant Cloud Free Tier will be used.
   - Retrieval must be done using the "function-tool" mechanism inside the agent.
   - All queries to Qdrant must be RAG-proper:
       - top_k retrieval
       - metadata return
       - similarity score

4. Chat Frontend (ChatKit)
   - Chatbot UI must be built using **ChatKit widget**.
   - ChatKit must communicate only with your FastAPI backend.
   - ChatKit must support:
       - selected-text-only mode (context override)
       - history-based chat threads
       - selected-text answers must behave like normal RAG answers (format, citations, clarity)

5. Backend (FastAPI → Hugging Face Deployment)
   - FastAPI must expose:
       - `/session` — create a new agent session
       - `/chat` — send user message + optional selected text
       - `/rag` — internal service to retrieve Qdrant chunks via MCP
   - Backend must be fully deployable on **Hugging Face Spaces**.
   - All env variables must be configured for OpenAI, Gemini, and Qdrant.
   - Backend logic must follow MCP context7 instructions for:
       - Agent initialization
       - Tool integration
       - Selected text handling

6. Frontend Deployment
   - Frontend (Docusaurus + ChatKit widget) will be deployed on **Vercel**.

7. Output Rules
   - The agent must ALWAYS cite chunks when answering.
   - The agent is NOT allowed to hallucinate.
   - If selected text is provided, the agent **will perform retrieval only on the selected text** using the same RAG mechanism, ensuring the answer format remains identical to normal RAG answers.


8. Quality Rules
   - All code generated must be self-contained, production-ready, and follow the project spec.
   - All tasks must be broken down using Spec-Kit Plus workflow:
        - `/sp.specify`
        - `/sp.clarify`
        - `/sp.plan`
        - `/sp.tasks`
        - `/sp.implement`
   - Use MCP context7 documentation at every step for guidance.

9. Validation
   - At every step, Claude Code must validate:
        - Qdrant schema compatibility
        - working MCP skills
        - correct agent JSON schema
        - ChatKit frontend script stability
        - selected text handling correctness
        - Gemini 2.5 Flash LLM connectivity
```

## Response Text
Updated the constitution file at `.specify/memory/constitution.md` with new architecture details, principles, and requirements for the AI-powered RAG chatbot project. The version was updated from 1.1.0 to 2.0.0 to reflect the major architectural changes. The new constitution includes sections for Agent Architecture, MCP Integration, Vector Database, Chat Frontend, Backend Architecture, Deployment Strategy, Output Rules, Quality Rules, and Validation Requirements.

## Outcome
The project constitution has been successfully updated to reflect the new AI-powered RAG chatbot architecture, replacing the previous book creation focus. The updated constitution provides clear guidance for implementing the system with the specified technologies and requirements.