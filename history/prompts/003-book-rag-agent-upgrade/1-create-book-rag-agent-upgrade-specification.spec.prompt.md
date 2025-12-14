---
id: 1
title: "create-book-rag-agent-upgrade-specification"
stage: spec
date: "2025-12-12"
surface: "agent"
model: "gemini-2.5-flash"
feature: "003-book-rag-agent-upgrade"
branch: "003-book-rag-agent-upgrade"
user: "user"
command: "/sp.specify"
labels: ["spec", "rag", "agent", "upgrade"]
links:
  spec: "specs/003-book-rag-agent-upgrade/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/003-book-rag-agent-upgrade/spec.md"
  - "specs/003-book-rag-agent-upgrade/checklists/requirements.md"
tests: []
---

# Book RAG Agent Upgrade Specification

## Summary
Created comprehensive specification for upgrading the existing chatbot to use OpenAI Agents SDK with Gemini 2.5 Flash model provider, Qdrant vector database, FastAPI backend, and ChatKit UI. The specification includes support for both normal RAG mode and selected text RAG mode.

## Changes
- Created detailed feature specification at specs/003-book-rag-agent-upgrade/spec.md
- Defined user scenarios with priorities (P1-P3)
- Specified functional requirements (FR-001 to FR-014)
- Defined success criteria (SC-001 to SC-005)
- Created quality checklist at specs/003-book-rag-agent-upgrade/checklists/requirements.md

## Prompt Text
```
-Book RAG Agent Upgrade Specification

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

context7 - resolve-library-id (MCP)(libraryName: "OpenAI Agents SDK")

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


If mode = "rag" → retrieve from Qdrant

If mode = "selected" → retrieve from user-selected text chunks

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
  "message": "user message",
  "selected_text": "... | null",
  "session_id": "uuid | null"
}

Logic
if selected_text:
    rag_mode = "selected"
else:
    rag_mode = "rag"


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
6. HF deployment config
```

## Response Text
Created comprehensive specification for the Book RAG Agent Upgrade feature. The specification includes:
- User scenarios with priorities (P1-P3) covering basic book queries, selected text queries, and session management
- Functional requirements (FR-001 to FR-014) detailing the system's behavior with OpenAI Agents SDK, Gemini model integration, RAG functionality, and API endpoints
- Success criteria (SC-001 to SC-005) with measurable outcomes for response accuracy, citation requirements, and system performance
- Quality checklist to ensure specification completeness

The specification defines the requirements for upgrading the existing chatbot to use OpenAI Agents SDK with Gemini 2.5 Flash, supporting both normal RAG mode and selected text RAG mode, with proper citations and session management.

## Outcome
The feature specification has been successfully created and validated. It provides clear requirements for implementing the Book RAG Agent Upgrade with OpenAI Agents SDK, enabling both normal RAG queries and selected text queries while maintaining proper citations and session management. The specification is ready for the planning phase.