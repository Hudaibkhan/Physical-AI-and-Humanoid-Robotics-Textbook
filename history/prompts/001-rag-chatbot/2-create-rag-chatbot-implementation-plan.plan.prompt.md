---
id: 2
title: Create RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-09
surface: agent
model: Claude-3.5-Sonnet
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: Claude user
command: /sp.plan
labels: [rag, chatbot, ai, digital-book, architecture]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-rag-chatbot/plan.md
 - specs/001-rag-chatbot/research.md
 - specs/001-rag-chatbot/data-model.md
 - specs/001-rag-chatbot/quickstart.md
 - specs/001-rag-chatbot/contracts/api-contract.yaml
tests:
 - specs/001-rag-chatbot/plan.md
---

## Prompt

Project: Add RAG Chatbot to Digital Book (Gemini + FastAPI + Qdrant)

Overall Goal:
Create a fully working Retrieval-Augmented Generation (RAG) chatbot integrated into my Docusaurus book website. The chatbot must use Gemini as the LLM, a free embedding model, Qdrant Cloud Free Tier for vector storage, and a FastAPI backend deployed on Render. The frontend chatbot widget will run on Vercel and appear on every page of the book. Book content format = Markdown. Expected concurrent users = 50.

---

Phase 1 — Setup (Accounts + Environment)
1. Create accounts:
   - Qdrant Cloud Free Tier
   - Google AI Studio (for Gemini API key)
   - Render.com (for FastAPI backend)
   - Vercel (for frontend deployment)
2. Create local project folders:
   mple popup widget in HTML + CSS + JS.
2. Features:
   - Open/close floating chatbot modal
   - Ask question → call backend → show answer
   - Dark/Light mode auto support
   - Text highlight → send selected text to backend as context

3. Integrate widget into Docusaurus book:
   - Add script tag in `docusaurus.config.js`
   - Include widget on ever- backend/
   - frontend/
   - book/
3. Install dependencies:
   - Python, FastAPI, Uvicorn
   - qdrant-client
   - free embedding model SDK (Gemini or OpenAI Mini)
   - OpenAI Agents/ChatKit SDK for orchestrating the chatbot

---

Phase 2 — Vector Database Setup (Qdrant)
1. Create a Qdrant Cloud Cluster (Free Tier).
2. Create a "book_chunks" collection with:
   - vector_size (according to embedding model)
   - cosine similarity
3. Write a script (`upload_chunks.py`) that:
   - Reads all Markdown book chapters
   - Splits them into chunks
   - Generates embeddings (free model)
   - Uploads them to Qdrant

Deliverables:
- Chunking logic
- Embedding generator
- Qdrant upload script

---

Phase 3 — FastAPI Backend
Create backend endpoints:

1. **POST /embed**
   - Accept text
   - Generate embedding (free model)
   - Return vector

2. **POST /search**
   - Query Qdrant using embedding
   - Return top matching chunks

3. **POST /ask-agent**
   - Input: user question + context + selected text (if any)
   - Fetch relevant chunks via search
   - Call Gemini LLM with:
       - retrieved chunks
       - user-selected text (priority)
   - Return final answer

4. Configure CORS for frontend access.

Deliverables:
- FastAPI project
- LLM caller (Gemini)
- Embedding module
- RAG pipeline

---

Phase 4 — Chatbot Agent Setup
1. Build agent using OpenAI Agents SDK or ChatKit SDK.
2. Agent tasks:
   - Accept question
   - Retrieve chunks via backend
   - Use Gemini LLM to generate answer
   - Reject out-of-book questions with:
     "This information is not in the book."

3. Integrate:
   - Gemini → final answer
   - Embedding model → vectorization
   - FastAPI → search + chunk retrieval

Deliverables:
- Agent configuration
- Routing logic
- RAG reasoning template
- Output formatting

---

Phase 5 — Frontend Chatbot Widget
1. Build a siy page

Deliverables:
- Chatbot widget
- API integration
- Highlight-to-answer support

---

Phase 6 — Deployment
1. Deploy FastAPI backend to Render (free)
2. Deploy book + widget to Vercel
3. Add environment vars:
   - GEMINI_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY
   - EMBEDDING_MODEL_KEY (free model)
4. Validate CORS + public API routes

Deliverables:
- Live backend URL
- Live book URL with embedded chatbot
- Working demo

---

Phase 7 — Final Testing & Optimization
1. Test:
   - RAG accuracy
   - Highlight-based answers
   - Out-of-book question rejection
   - Slow/fast networks
   - 50 concurrent user simulation
2. Fix:
   - Slow embedding calls
   - Chunk sizes
   - Gemini prompt tuning

Deliverables:
- Final working demo
- Performance improvements

---

Completion Criteria:
✔ RAG chatbot embedded inside book
✔ Gemini-powered reasoning
✔ Free embedding model implemented
✔ Qdrant vector search working
✔ FastAPI backend functioning
✔ Popup chatbot widget live
✔ Highlight-based answering supported
✔ Deployed on Render + Vercel
✔ Handles 50 concurrent users

## Response snapshot

# Implementation Plan: RAG Chatbot for Digital Book Website

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-09 | **Spec**: [specs/001-rag-chatbot/spec.md](specs/001-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Retrieval-Augmented Generation (RAG) chatbot for a digital book website that allows users to ask questions about book content and receive answers generated by Google Gemini LLM based on the book material. The system includes a FastAPI backend with Qdrant vector storage, a frontend widget for Docusaurus integration, and supports text highlighting for context-specific answers.

## Technical Context

**Language/Version**: Python 3.11 (backend), JavaScript/ES6 (frontend), Markdown format for book content
**Primary Dependencies**: FastAPI, qdrant-client, google-generativeai, Pydantic, Docusaurus v3
**Storage**: Qdrant Cloud Free Tier (vector database for embeddings), potential local storage for configuration
**Testing**: pytest (backend), Jest (frontend - if applicable)
**Target Platform**: Web application (cloud deployment with Render/Vercel)
**Project Type**: Web (separate backend and frontend applications)
**Performance Goals**: <5 second response time for RAG queries, support 50 concurrent users
**Constraints**: Must use free tier services (Qdrant, Render), Gemini API usage limits, must reject out-of-book questions
**Scale/Scope**: 50 concurrent users, book content size dependent on Qdrant free tier limits

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution:
1. **Spec-Driven Writing**: ✅ Plan follows specification from spec.md
2. **Technical Reliability**: ✅ Technology stack (FastAPI, Qdrant, Gemini) is established and verifiable
3. **Beginner-Friendly + Professional Tone**: ✅ Plan includes clear documentation and structured approach
4. **Documentation Quality**: ✅ Following high-quality documentation standards
5. **Consistency**: ✅ Plan maintains consistent structure with other project artifacts

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── embedding.py
│   │   ├── rag.py
│   │   └── chat.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── qdrant_service.py
│   │   ├── gemini_service.py
│   │   └── rag_service.py
│   ├── api/
│   │   ├── v1/
│   │   │   ├── router.py
│   │   │   ├── endpoints/
│   │   │   │   ├── embed.py
│   │   │   │   ├── search.py
│   │   │   │   └── ask_agent.py
│   │   │   └── schemas/
│   │   │       ├── embedding.py
│   │   │       ├── search.py
│   │   │       └── chat.py
│   ├── config/
│   │   └── settings.py
│   └── main.py
├── scripts/
│   └── upload_chunks.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
└── Dockerfile

frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.jsx
│   │   ├── ChatModal.jsx
│   │   ├── Message.jsx
│   │   └── InputArea.jsx
│   ├── services/
│   │   └── api.js
│   ├── styles/
│   │   └── chatbot.css
│   └── utils/
│       └── textSelection.js
├── index.html
├── package.json
└── widget.js

book/
├── docs/
│   └── [existing book content]
├── docusaurus.config.js
└── static/
    └── js/
        └── chatbot-widget.js
```

**Structure Decision**: The project will be structured as a web application with separate backend and frontend components. The backend will be a FastAPI application deployed to Render, while the frontend will be a JavaScript widget integrated into the existing Docusaurus book website deployed on Vercel. This separation allows for independent scaling and maintenance of each component.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple services architecture | Required for proper separation of concerns between frontend, backend, and vector database | Single service would create tight coupling and deployment complexity |
| Third-party dependencies (Qdrant, Gemini) | Essential for RAG functionality and LLM capabilities | Building in-house alternatives would require significant development time and resources |

## Outcome

- ✅ Impact: Created comprehensive implementation plan for RAG chatbot with detailed architecture
- 🧪 Tests: Plan includes testing strategy and validation approaches
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and API contracts in specs/001-rag-chatbot/
- 🔁 Next prompts: Ready for /sp.tasks to create implementation tasks
- 🧠 Reflection: Plan addresses all requirements from the specification and provides a clear roadmap for implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All planning requirements met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin task creation with /sp.tasks