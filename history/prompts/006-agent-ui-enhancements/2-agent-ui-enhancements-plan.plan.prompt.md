---
id: 2
title: agent-ui-enhancements-plan
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 006-agent-ui-enhancements
branch: 006-agent-ui-enhancements
user: Hudaibkhan
command: /sp.plan
labels: ["implementation-plan", "architecture", "llm-routing", "greeting-detection", "ui-animation", "rag-metadata-suppression"]
links:
  spec: specs/006-agent-ui-enhancements/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-agent-ui-enhancements/plan.md
 - specs/006-agent-ui-enhancements/research.md
 - specs/006-agent-ui-enhancements/data-model.md
 - specs/006-agent-ui-enhancements/quickstart.md
 - specs/006-agent-ui-enhancements/contracts/api-contract.md
tests:
 - N/A (planning phase)
---

## Prompt

User requested: `/sp.plan — Implementation Plan`

Full user-provided planning input:

## Title
Step-by-Step Plan to Update Agent Behavior, LLM Routing, and Thinking-State UI Animation

---

## Goal

Implement the `/sp.specify` requirements by updating:

- Agent behavior (greetings, formatting, RAG safety)
- LLM routing with silent fallback
- Frontend UI thinking-state animation

All changes must work within the existing **FastAPI + OpenAI Agents SDK** architecture and be implementable using **Claude Code (SpecKit Plus)**.

---

## Phase 1 — Analysis & Inventory

### Step 1.1: Audit Current Backend Structure
- Locate the agent definition inside `/fastapi`
- Identify:
  - Agent system prompt / instructions
  - RAG retrieval call
  - LLM invocation logic
  - Error handling for LLM failures (429, quota)

### Step 1.2: Audit Frontend Agent UI
- Locate the chat UI component
- Identify:
  - Message send handler
  - Response receive / streaming logic
  - Loading or placeholder states (if any)

---

## Phase 2 — Agent Behavior Updates

### Step 2.1: Add Greeting Intent Detection
- Introduce lightweight intent classification
- Detect greetings **before** RAG execution
- Bypass RAG when greeting intent is detected

### Step 2.2: Update Agent System Instructions
- Enforce:
  - Structured markdown output
  - Clear headings and bullet points
  - Professional instructor tone

### Step 2.3: Suppress RAG Metadata
- Remove any logic that injects:
  - Chunk counts
  - Chunk IDs
  - Source annotations
- Ensure final response is post-processed if needed

### Step 2.4: Handle Weak or Ambiguous Queries
- Add fallback reasoning when context is incomplete
- Prefer high-level explanations over refusals
- Ask clarifying questions only when necessary

---

## Phase 3 — LLM Routing & Fallback

### Step 3.1: Define Model Priority
- Primary: Gemini (free tier)
- Secondary: Groq (free)

### Step 3.2: Implement Silent Fallback Logic
- Detect Gemini errors:
  - HTTP 429
  - RESOURCE_EXHAUSTED
- Retry request using Groq automatically
- Preserve original prompt and system instructions

### Step 3.3: Normalize Responses Across Models
- Ensure Gemini and Groq outputs pass through:
  - Same formatter
  - Same post-processing layer

---

## Phase 4 — Frontend Thinking-State Animation

### Step 4.1: Define Thinking State
- Introduce `isThinking` boolean state
- Set to `true` on message send

### Step 4.2: Implement Dot-Waving Animation Component
- Create lightweight UI component:
  - Three dots
  - Wave / pulse animation
  - Loop ~1 second

### Step 4.3: Connect Animation to Agent Response
- Stop animation on:
  - First streamed token OR
  - Full response received

---

## Phase 5 — Backend ↔ Frontend Coordination

### Step 5.1: Emit Response-Start Signal
- Ensure backend streaming or response-ready event is detectable
- No API contract changes

### Step 5.2: UI Listener Integration
- Frontend listens for response start
- Toggles `isThinking = false`

---

## Phase 6 — Testing & Validation

### Step 6.1: Behavior Testing
- Test greetings
- Test knowledge queries
- Test ambiguous queries

### Step 6.2: LLM Fallback Testing
- Simulate Gemini 429 error
- Verify Groq fallback activates silently

### Step 6.3: UI/UX Testing
- Verify thinking animation appears on send
- Verify animation stops correctly
- Ensure no UI freezing

---

## Phase 7 — Cleanup & Documentation

### Step 7.1: Remove Debug Logs
- Remove any temporary console logs
- Ensure no internal data is exposed

### Step 7.2: Update Internal Docs
- Document:
  - Agent behavior rules
  - Model routing logic
  - UI thinking-state behavior

---

## Final Success Criteria

- ✅ Agent responds to greetings correctly
- ✅ Clean, structured markdown responses
- ✅ No RAG metadata leakage
- ✅ Gemini → Groq fallback works silently
- ✅ Smooth dot-waving thinking animation
- ✅ No breaking changes to APIs
- ✅ Fully aligned with `/sp.specify`

---

## Response snapshot

Successfully created comprehensive implementation plan with complete Phase 0 (Research) and Phase 1 (Design) artifacts.

### Planning Process Executed

**1. Setup & Context Loading:**
- Ran `.specify/scripts/powershell/setup-plan.ps1` to initialize planning structure
- Loaded feature specification from `specs/006-agent-ui-enhancements/spec.md`
- Loaded constitution from `.specify/memory/constitution.md`
- Loaded plan template from `specs/006-agent-ui-enhancements/plan.md`

**2. Architecture Audit (Parallel Exploration):**
- **Backend Audit Agent:** Comprehensively explored `fastapi_app/` directory
  - Identified OpenAI Agents SDK integration with LiteLLM
  - Located agent definition in `agent.py` (book_rag_agent with Gemini 2.5 Flash)
  - Mapped RAG retrieval in `connection.py` (Qdrant + Cohere embeddings)
  - Analyzed LLM invocation via `Runner.run()` pattern
  - Documented existing error handling and session management
  - Confirmed no streaming support (HTTP POST/response pattern)
  - Cataloged 7 API endpoints including `/chat`

- **Frontend Audit Agent:** Comprehensively explored `src/` directory
  - Located chat UI in `src/theme/Root.tsx`
  - Identified message send via native `fetch()` API
  - Analyzed response handling (complete JSON, no streaming)
  - Found loading state implementation (`isLoading` with ⏳ emoji)
  - Confirmed React 18 + Docusaurus 3 + TypeScript stack
  - Documented inline CSS styling approach

**3. Phase 0: Research & Technology Decisions:**
Generated `research.md` (15 comprehensive technology decisions):

1. **Greeting Detection:** Pattern matching with early exit (<1ms)
2. **LLM Routing:** Try-catch wrapper with silent Gemini → Groq fallback
3. **Response Formatting:** System prompt enhancement + post-processing safety net
4. **RAG Metadata Suppression:** Three-stage defense (prompt, regex, validation)
5. **Thinking Animation:** CSS keyframe animation (60fps) + React state
6. **Streaming:** Deferred (not MVP) - acceptable latency with animation
7. **Error Handling:** Graceful degradation with user-friendly messages
8. **Testing:** Manual testing + pytest integration tests
9. **Environment Configuration:** Extend existing .env pattern with feature flags
10. **Performance:** Target latencies documented (greeting <1ms, query 3-5s)
11. **Deployment:** No infrastructure changes (Hugging Face + Vercel)
12. **Compliance with Constitution:** Full alignment validated
13. **Summary of Decisions:** Technology matrix created
14. **Open Questions:** All NEEDS CLARIFICATION items resolved
15. **Phase 0 Complete:** Ready for Phase 1

**4. Phase 1: Design & Contracts:**

**Artifact 1: data-model.md** - Defined 5 runtime entities:
- **GreetingPattern:** Static configuration set for intent detection
- **ThinkingState:** Frontend UI state (isThinking boolean + startedAt timestamp)
- **LLMProvider:** Runtime configuration with primary/fallback logic
- **ResponseEvent:** Backend event signal (future enhancement placeholder)
- **AgentMessage:** Extended DTO with metadata suppression validation

Key insights:
- No database changes required (all runtime constructs)
- Existing SessionManager preserved
- Pydantic validator enforces metadata prohibition

**Artifact 2: contracts/api-contract.md** - Complete API specification:
- Modified `/chat` endpoint behavior documented
- Request schema unchanged (backward compatible)
- Response schema extended (removed `citations`, kept `source_chunks` internal)
- Greeting response example with empty source_chunks
- Technical query example with formatted markdown
- Error response patterns (400, 500, 503 with graceful degradation)
- Backend implementation details:
  - Greeting detection logic location and code pattern
  - LLM fallback logic with try-catch wrapper
  - Metadata suppression with regex patterns
- Frontend integration changes
- Performance characteristics (latency budget, rate limits)
- Testing contract with curl examples

**Artifact 3: quickstart.md** - Step-by-step implementation guide:
- 6 implementation phases with code samples
- Phase 1: Backend Greeting Detection (30 min)
- Phase 2: Backend LLM Fallback (45 min)
- Phase 3: Backend Response Formatting (30 min)
- Phase 4: Backend Metadata Suppression (30 min)
- Phase 5: Frontend Thinking Animation (45 min)
- Phase 6: Testing & Validation (30 min)
- Total estimated time: 3-4 hours
- Troubleshooting guide with common issues
- Manual testing checklist
- Integration test examples

**5. plan.md Completion:**

Filled comprehensive plan.md with:
- **Summary:** Core technical approach documented
- **Technical Context:** Python 3.11+, TypeScript/React 18, FastAPI, OpenAI Agents SDK
- **Constitution Check:** All 5 principles validated (✅ PASS)
- **Project Structure:** Web application (backend + frontend) with modular code organization
- **Complexity Tracking:** Empty (no violations)
- **Phase 0 Status:** ✅ Complete (research.md with 15 decisions)
- **Phase 1 Status:** ✅ Complete (3 design artifacts)
- **Implementation Roadmap:**
  - Phase 2: `/sp.tasks` (20-25 atomic tasks expected)
  - Phase 3: `/sp.implement` (TDD cycle)
  - Phase 4: Validation & Deployment
- **Risk Analysis:** 3 risks identified with mitigation strategies
- **Dependencies:** Groq API (new), Gemini/Qdrant/Cohere (existing)
- **Deployment Plan:** Hugging Face Spaces (backend), Vercel (frontend)
- **Monitoring:** Backend logging strategy for greeting detection, LLM fallback, metadata detection
- **Testing Strategy:** Manual checklist + pytest integration tests
- **Rollback Plan:** Git revert + feature flags for partial rollback
- **Success Criteria:** 8 quality criteria from spec.md mapped

**6. Agent Context Update:**
- Ran `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`
- Updated CLAUDE.md with feature context (placeholder values captured, expected for unfilled plan)

**7. Constitution Re-Check (Post-Design):**
✅ All 5 principles maintained:
- Agent-Driven Architecture: Logic integrated with OpenAI Agents SDK
- MCP Integration: RAG tools unchanged
- Production-Ready: Free tier, existing deployment model
- Response Quality: Enhanced formatting, no hallucination
- System Consistency: Normalized outputs

### Key Design Decisions

**1. No Database Changes:**
- All entities are runtime constructs or configuration constants
- In-memory state management via existing SessionManager

**2. Backward Compatible API:**
- Request schema unchanged
- Response schema extended (removed optional `citations`, kept `source_chunks` internal)
- HTTP status codes unchanged

**3. Modular Code Organization:**
```
fastapi_app/
├── greeting_detector.py    # Single responsibility: greeting detection
├── llm_router.py            # Single responsibility: LLM routing + fallback
├── formatters.py            # Single responsibility: response cleaning
```

**4. Defense in Depth:**
- Greeting Detection: Explicit check before RAG
- Metadata Suppression: System prompt + regex + validation
- Error Handling: Try-catch + fallback + user-friendly messages
- LLM Routing: Primary + fallback + logging

### Files Generated

**Phase 0:**
- `specs/006-agent-ui-enhancements/research.md` (15 technology decisions, 6,500+ words)

**Phase 1:**
- `specs/006-agent-ui-enhancements/data-model.md` (5 entities, state transitions, validation rules)
- `specs/006-agent-ui-enhancements/contracts/api-contract.md` (complete API specification, examples, testing)
- `specs/006-agent-ui-enhancements/quickstart.md` (6-phase implementation guide, 4,500+ words)
- `specs/006-agent-ui-enhancements/plan.md` (comprehensive plan with roadmap, risks, deployment)

**Agent Context:**
- Updated `CLAUDE.md` with feature context (placeholder values as expected for template-based plan)

### Next Steps

**Command to run:** `/sp.tasks`

**Expected output:** `tasks.md` with 20-25 atomic, testable implementation tasks organized by:
1. Backend: Greeting Detection (4-5 tasks)
2. Backend: LLM Routing & Fallback (5-6 tasks)
3. Backend: Response Formatting (3-4 tasks)
4. Backend: Metadata Suppression (3-4 tasks)
5. Frontend: Thinking Animation (4-5 tasks)
6. Integration Testing (3-4 tasks)
7. Deployment & Validation (2-3 tasks)

**Estimated implementation time after tasks:** 3-4 hours implementation + 1 hour testing = 4-5 hours total

## Outcome

- ✅ Impact: Comprehensive implementation plan with complete Phase 0 (Research) and Phase 1 (Design) artifacts ready for task generation
- 🧪 Tests: N/A (planning phase - test strategy documented for implementation phase)
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, quickstart.md, api-contract.md)
- 🔁 Next prompts: `/sp.tasks` (generate atomic implementation tasks) → `/sp.implement` (execute tasks)
- 🧠 Reflection: Planning achieved clarity through parallel architecture exploration (backend + frontend agents), comprehensive technology research (15 decisions), and detailed design artifacts (data models, API contracts, quickstart guide). Zero constitution violations, backward-compatible API design, and clear separation of concerns enable smooth implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all architecture audits successful, constitution checks passed, design artifacts complete
- Graders run and results (PASS/FAIL): Manual validation against plan template requirements - PASS (all mandatory sections filled, no NEEDS CLARIFICATION remaining)
- Prompt variant (if applicable): Standard `/sp.plan` workflow with comprehensive user-provided phase outline
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to generate atomic implementation tasks based on this plan, then `/sp.implement` for TDD execution
