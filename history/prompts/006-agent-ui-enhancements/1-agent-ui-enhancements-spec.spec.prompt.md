---
id: 1
title: agent-ui-enhancements-spec
stage: spec
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 006-agent-ui-enhancements
branch: 006-agent-ui-enhancements
user: Hudaibkhan
command: /sp.specify
labels: ["specification", "agent-behavior", "ui-enhancement", "llm-routing", "rag-metadata-suppression"]
links:
  spec: specs/006-agent-ui-enhancements/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-agent-ui-enhancements/spec.md
 - specs/006-agent-ui-enhancements/checklists/requirements.md
tests:
 - N/A (specification phase)
---

## Prompt

User requested: `/sp.specify — Full Agent & Frontend UI Update Specification`

Full feature description:

## Title
Physical AI & Humanoid Robotics Agent
Behavior, Formatting, LLM Routing, and Thinking-State UI Animation Update

---

## Objective

Enhance both the **agent behavior** and **frontend UI experience** so that:

- The agent responds correctly to greetings
- All answers are structured and professional
- Internal RAG metadata is never exposed
- Gemini (free tier) is used first, with Groq as a silent fallback
- The frontend shows a **smooth "thinking" animation** while the LLM is processing
- All changes remain compatible with the existing FastAPI + OpenAI Agents SDK setup

---

## Environment & Constraints

### Backend
- Framework: **FastAPI**
- Agent Runtime: **OpenAI Agents SDK**
- RAG: Existing vector store + retriever (unchanged)
- LLMs:
  - Primary: Gemini (free tier)
  - Fallback: Groq (free)
- Spec System: **SpecKit Plus**
- Implementation Tool: **Claude Code**
- Scope: `/fastapi`

### Frontend
- Agent UI: Existing chat interface
- Transport: HTTP / Streaming (as currently implemented)
- No paid UI libraries allowed

---

## Key Requirements from User Input

1. **Greeting & Small-Talk Handling**
   - Detect greeting intent before RAG execution
   - Respond politely without referencing textbooks or sources
   - Examples: hi, hello, hey, salam, assalam o alaikum

2. **Structured Output Formatting (Mandatory)**
   - Clean markdown with headings
   - Bullet points or numbered lists
   - Short paragraphs
   - Template: Topic Title > Brief explanation > Key Points > Why It Matters

3. **RAG Metadata Suppression (Critical)**
   - No chunk counts, chunk IDs, source metadata, RAG identifiers
   - Responses must appear fully authored

4. **LLM Routing (Gemini → Groq Fallback)**
   - Gemini first (free tier)
   - Automatically retry with Groq on quota exceeded
   - Fallback must be silent (no user messages about switching)

5. **Frontend UI — Thinking State Animation**
   - Three-dot wave animation while LLM is processing
   - Animation appears on request send, disappears on first response token
   - Smooth, subtle, neutral color
   - No "Loading..." text or spinners

6. **Backend ↔ Frontend Coordination**
   - Backend emits "response started" signal
   - Frontend listens and removes animation immediately

7. **Integration Rules**
   - Preserve existing FastAPI endpoints
   - Preserve existing RAG pipeline
   - No paid dependencies
   - No backend error exposure to UI

8. **Agent Persona**
   - Robotics instructor
   - Friendly and calm
   - Confident and precise
   - Clean and professional

## Response snapshot

Successfully created comprehensive specification document with:

### 1. Feature Structure Setup
- Created feature branch: `006-agent-ui-enhancements`
- Generated feature directory: `specs/006-agent-ui-enhancements/`
- Initialized spec.md with complete template-based specification

### 2. User Scenarios & Testing (5 prioritized stories)

**P1 Stories (Critical):**
- **Greeting Interaction**: Detect greeting intent before RAG, respond naturally without metadata exposure
- **Professional Response Formatting**: Structured markdown with headings, key points, "Why It Matters" sections
- **Metadata Suppression**: Strip all RAG internal details (chunk IDs, sources) from user-facing responses

**P2 Stories (Important):**
- **Seamless LLM Fallback**: Gemini → Groq silent failover without user awareness
- **Visual Thinking Indicator**: Three-dot wave animation during processing, removed on first token

Each story includes:
- Plain language description
- Priority justification
- Independent test criteria
- Given-When-Then acceptance scenarios

### 3. Functional Requirements (28 requirements organized by concern)

**Agent Behavior (FR-001 to FR-008):**
- Greeting detection before RAG execution
- Markdown-structured responses with consistent format
- RAG metadata stripping
- Robotics instructor persona

**LLM Routing & Fallback (FR-009 to FR-013):**
- Gemini primary, Groq fallback
- Silent error handling
- Server-side logging without user exposure

**Frontend UI - Thinking State (FR-014 to FR-019):**
- Three-dot wave animation specs
- Animation timing (1s loop)
- Non-blocking UI during thinking state

**Backend-Frontend Coordination (FR-020 to FR-023):**
- Response start signal emission
- Streaming and non-streaming handling

**Integration Constraints (FR-024 to FR-028):**
- Preserve existing FastAPI endpoints
- Maintain RAG pipeline
- No paid dependencies
- No error exposure

### 4. Success Criteria (8 measurable outcomes)

All criteria are technology-agnostic and verifiable:
- **SC-001**: 100% greeting detection accuracy
- **SC-002**: 100% response formatting compliance
- **SC-003**: 0% RAG metadata leakage
- **SC-004**: <2s fallback completion
- **SC-005**: <100ms animation start, <50ms animation stop
- **SC-006**: Professional perception (user feedback)
- **SC-007**: 3-5s response time with progress indication
- **SC-008**: 0% backend error exposure

### 5. Edge Cases Identified
- Mixed greeting + question in one message
- Ambiguous greetings ("sup", "yo")
- Double-failure (both Gemini and Groq fail)
- WebSocket drops during streaming
- Backend error instead of response signal
- Rapid-fire messages
- Embedded RAG metadata in content

### 6. Quality Validation

Created comprehensive checklist at `specs/006-agent-ui-enhancements/checklists/requirements.md` with validation results:

✅ **Content Quality**: All sections pass - no implementation details, user-focused, accessible language
✅ **Requirement Completeness**: All FRs testable, no clarification markers needed, edge cases covered
✅ **Feature Readiness**: Ready for planning phase

### 7. Key Assumptions Documented
- Gemini/Groq free tier sufficiency and comparability
- FastAPI streaming support
- OpenAI Agents SDK LLM provider flexibility
- Vector store conditional bypass capability
- Modern browser CSS animation support

## Outcome

- ✅ Impact: Comprehensive, implementation-ready specification created for agent behavior and UI enhancements
- 🧪 Tests: N/A (specification phase - test scenarios defined in acceptance criteria)
- 📁 Files: 2 files created (spec.md, checklists/requirements.md)
- 🔁 Next prompts: `/sp.clarify` (optional if clarifications needed) or `/sp.plan` (proceed to implementation planning)
- 🧠 Reflection: Specification achieves clarity by separating concerns (agent behavior, LLM routing, frontend UI, backend coordination) and provides independently testable user stories with clear priorities

## Evaluation notes (flywheel)

- Failure modes observed: None - all quality checks passed
- Graders run and results (PASS/FAIL): Manual checklist validation - PASS (all 14 checklist items)
- Prompt variant (if applicable): Standard /sp.specify workflow with comprehensive user input
- Next experiment (smallest change to try): Proceed to `/sp.plan` to design implementation architecture, or run `/sp.clarify` if any assumptions need user confirmation
