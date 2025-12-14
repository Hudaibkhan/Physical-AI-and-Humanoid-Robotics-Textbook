---
id: 2
title: "create-book-rag-agent-upgrade-plan"
stage: plan
date: "2025-12-12"
surface: "agent"
model: "gemini-2.5-flash"
feature: "003-book-rag-agent-upgrade"
branch: "003-book-rag-agent-upgrade"
user: "user"
command: "/sp.plan"
labels: ["plan", "rag", "agent", "upgrade"]
links:
  spec: "specs/003-book-rag-agent-upgrade/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/003-book-rag-agent-upgrade/plan.md"
  - "specs/003-book-rag-agent-upgrade/research.md"
  - "specs/003-book-rag-agent-upgrade/data-model.md"
  - "specs/003-book-rag-agent-upgrade/contracts/api-contract.yaml"
  - "specs/003-book-rag-agent-upgrade/quickstart.md"
  - ".specify/memory/agent-context-claude.md"
tests: []
---

# Book RAG Agent Upgrade Implementation Plan

## Summary
Created comprehensive implementation plan for upgrading the existing chatbot to use OpenAI Agents SDK with Gemini 2.5 Flash model provider, supporting both normal RAG mode and selected text RAG mode. The plan includes backend architecture with connection.py, agent.py, and app.py, as well as frontend integration with ChatKit.

## Changes
- Created implementation plan at specs/003-book-rag-agent-upgrade/plan.md
- Created research document with technical approaches
- Defined data models for the new system
- Created API contract using OpenAPI specification
- Created quickstart guide for developers
- Updated agent context with new technology information

## Prompt Text
```
-Updated Plan for Book RAG Agent Upgrade

## 🎯 Goal

Upgrade the existing chatbot to a **Book RAG Agent** that:

- Uses **OpenAI Agents SDK**
- Uses **Gemini 2.5 Flash** as external model
- Answers **only from the book** (Qdrant dataset or selected text)
- Supports **Normal RAG mode** and **Selected Text RAG mode**
- Uses **MCP / context7** only for guidance on agent/tool structure
- Uses **ChatKit frontend** + **FastAPI backend** deployed on **Hugging Face Spaces**

---

## 🧩 Phase 1 — MCP + Context7 Guidance

### 1.1 Call MCP / context7

**Command:**

context7 - resolve-library-id (MCP)(libraryName: "OpenAI Agents SDK")

markdown
Copy code

**Purpose:**

- Retrieve documentation on Agents SDK
- How to define agents and function tools
- How to attach external models (Gemini)
- How to structure `RunConfig` and `Runner.run()`

> **Note:** MCP does not provide execution tools — only reference docs.

### 1.2 Review Returned Docs

Understand:

- Agent creation syntax
- Tool registration (`@function_tool`)
- External LLM integration
- Handling sessions and outputs

Use this info to generate correct agent code.

---

## ⚙️ Phase 2 — Build Agent with OpenAI Agents SDK

### 2.1 Create Agent

- **Name:** `book_rag_agent`
- **Model:** Gemini 2.5 Flash (external provider)
- **Instructions:**
  - Answer only from Qdrant/book content or selected text
  - Never hallucinate
  - Always cite retrieved chunks

**Tools:**

```python
rag_query(query: str, mode: str, top_k: int)
mode="rag" → query Qdrant

mode="selected" → query only selected text chunks

All retrieval logic is implemented by your code, not MCP

2.2 Retrieval Logic
Normal RAG mode:

Convert user query → embeddings

Retrieve top_k similar chunks from Qdrant

Return chunks → agent produces final answer

Selected Text mode:

Split selected text → compute embeddings

Find top-k matching chunks

Return chunks → agent produces answer in same format as RAG mode

🔧 Phase 3 — Backend (FastAPI)
3.1 API Endpoints
Endpoint: /chat

Body:

json
Copy code
{
  "message": "...",
  "selected_text": "... | null",
  "session_id": "... | null"
}
Logic:

If selected_text exists → mode = "selected"

Else → mode = "rag"

3.2 Agent Execution
Use Agents SDK Runner:

python
Copy code
Runner.run(agent, message, run_config=config, metadata={...})
Return final_output with citations

🧱 Phase 4 — Frontend (ChatKit)
Embed ChatKit widget in Docusaurus

Features:

Text input box

Selected text support

Message history

Display citations

Communicates only with HF backend /chat endpoint

📦 Phase 5 — Deployment
5.1 Hugging Face (Backend)

Files:

connection.py → Gemini, embeddings, Qdrant client setup

agent.py → define book_rag_agent + rag_query tool

app.py → FastAPI endpoints

requirements.txt

5.2 Vercel (Frontend)
Docusaurus + ChatKit widget

Points to HF backend /chat

🧹 Phase 6 — Cleanup
Delete old/unnecessary chatbot files

Keep project clean before generating new code

🛠️ Phase 7 — Testing
Unit Tests

RAG mode queries → correct chunks + citations

Selected text mode → uses only selected chunks

No hallucinations

End-to-End: ChatKit → HF backend → Agent → Qdrant → response

Ensure output format consistency

🔑 Notes
MCP / context7 only guides code generation; does not perform retrieval

All tools (Qdrant + selected-text search) must be implemented in connection.py + agent.py

Agent always wraps retrieval logic via function tool using Agents SDK
```

## Response Text
Created comprehensive implementation plan for the Book RAG Agent Upgrade. The plan includes:

- Technical context with resolved unknowns from research.md
- Constitution check verifying compliance with project principles
- Implementation architecture for backend (connection.py, agent.py, app.py)
- Agent implementation details for book_rag_agent and rag_query function tool
- Frontend integration with ChatKit
- Deployment architecture for Hugging Face and Vercel
- Implementation steps and validation requirements

The plan addresses all requirements from the specification, including using OpenAI Agents SDK with Gemini 2.5 Flash, supporting both normal RAG mode and selected text RAG mode, and ensuring proper citations and no hallucination. The backend architecture separates concerns with dedicated files for connections, agent logic, and API endpoints.

## Outcome
The implementation plan has been successfully created and validated against the project constitution. It provides a clear roadmap for implementing the Book RAG Agent Upgrade with OpenAI Agents SDK, including all necessary components for both RAG modes, proper citations, and deployment to Hugging Face Spaces. The plan is ready for the tasks phase (/sp.tasks).