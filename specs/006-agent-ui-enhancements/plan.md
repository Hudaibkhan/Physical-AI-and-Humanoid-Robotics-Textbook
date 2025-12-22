# Implementation Plan: Agent Behavior & UI Enhancement

**Branch**: `006-agent-ui-enhancements` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-agent-ui-enhancements/spec.md`

---

## Summary

Enhance the Physical AI & Humanoid Robotics chatbot agent with intelligent greeting detection, professional response formatting, LLM fallback routing (Gemini → Groq), RAG metadata suppression, and smooth frontend thinking animations. All changes integrate seamlessly with existing FastAPI + OpenAI Agents SDK architecture without breaking changes.

**Core Technical Approach:**
- **Greeting Detection:** Pre-RAG pattern matching (string comparison, <1ms)
- **LLM Routing:** Try-catch wrapper with silent Gemini → Groq fallback
- **Response Formatting:** Enhanced system prompt + optional post-processing safety net
- **Metadata Suppression:** Three-stage defense (system prompt, regex cleaning, validation)
- **Thinking Animation:** CSS keyframe animation (60fps) + React state management

---

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React 18 (frontend)
**Primary Dependencies**:
- Backend: FastAPI 0.115+, openai-agents 0.0.14, litellm 1.51+, qdrant-client 1.12+
- Frontend: React 18, Docusaurus 3.0, TypeScript
- LLMs: Google Gemini 2.5 Flash (primary), Groq Llama 3.1 70B (fallback)

**Storage**: Qdrant vector database (existing, unchanged), in-memory session management
**Testing**: Manual testing + pytest integration tests
**Target Platform**: Hugging Face Spaces (backend), Vercel (frontend)
**Project Type**: Web application (FastAPI backend + React frontend)
**Performance Goals**: <1ms greeting detection, 3-5s total response time, <100ms animation start/stop
**Constraints**: Zero paid dependencies, no breaking API changes, free tier LLM quotas (Gemini 15 RPM, Groq 30 RPM)
**Scale/Scope**: ~500 lines of new code, 5 new files, 3 modified files

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Agent-Driven Architecture
✅ **PASS** - All logic runs through OpenAI Agents SDK with Gemini 2.5 Flash as primary model
- Greeting detection occurs before agent call, not instead of agent
- Agent instructions enhanced, not bypassed
- LLM routing extends existing Runner.run() pattern

### MCP Integration & Validation
✅ **PASS** - Existing MCP tools (RAG, Qdrant, embeddings) preserved without modification
- No changes to RAG retrieval mechanism
- No changes to Qdrant vector search
- No changes to embedding generation

### Production-Ready Implementation
✅ **PASS** - All code follows existing patterns, deployable to current infrastructure
- Free tier APIs only (Gemini, Groq)
- No new infrastructure dependencies
- Environment variable configuration (same as existing .env pattern)

### Agent Response Quality
✅ **PASS** - Enhanced formatting and metadata suppression improve quality
- Structured markdown output enforced
- RAG citations still retrieved (for audit) but not displayed
- No hallucination (context-only responses maintained)

### System Consistency
✅ **PASS** - Identical formatting across all LLM providers and response modes
- Gemini and Groq outputs normalized through same formatter
- Greeting and technical responses follow same markdown structure
- Error handling provides consistent user-friendly messages

**Constitution Status**: ✅ All principles maintained. No violations or complexity injections.

---

## Project Structure

### Documentation (this feature)

```text
specs/006-agent-ui-enhancements/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Technology decisions ✅
├── data-model.md        # Phase 1 output: Entity definitions ✅
├── quickstart.md        # Phase 1 output: Implementation guide ✅
├── contracts/           # Phase 1 output: API specifications ✅
│   └── api-contract.md
└── tasks.md             # Phase 2 output: /sp.tasks command (NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (existing)
fastapi_app/                      # Backend
├── app.py                        # FastAPI application (MODIFY)
├── agent.py                      # Agent definition (MODIFY)
├── connection.py                 # ConnectionManager, SessionManager (MODIFY)
├── greeting_detector.py          # NEW: Greeting pattern matching
├── llm_router.py                 # NEW: LLM routing and fallback
├── formatters.py                 # NEW: Response formatting and metadata suppression
└── requirements.txt              # No new dependencies

src/                              # Frontend
└── theme/
    └── Root.tsx                  # Chat UI component (MODIFY)

.env.example                      # NEW: Add GROQ_API_KEY

tests/                            # Integration tests (NEW)
├── test_greeting_detection.py
├── test_llm_fallback.py
└── test_metadata_suppression.py
```

**Structure Decision**: Web application structure maintained. Backend changes localized to `fastapi_app/` directory. Frontend changes localized to `src/theme/Root.tsx`. No architectural refactoring needed.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations detected. This section intentionally left empty.

---

## Phase 0: Research & Technology Decisions

**Status**: ✅ Complete

**Artifacts Generated**:
- `research.md` - 15 technology decisions documented with rationale

**Key Decisions**:
1. **Greeting Detection:** Pattern matching (not ML)
2. **LLM Routing:** Try-catch wrapper with Gemini → Groq fallback
3. **Response Formatting:** System prompt + optional post-processing
4. **Metadata Suppression:** Three-stage defense (prompt, regex, validation)
5. **Thinking Animation:** CSS keyframes + React state (no libraries)
6. **Streaming:** Deferred to future (not MVP)
7. **Error Handling:** Graceful degradation with user-friendly messages
8. **Testing:** Manual + pytest integration tests
9. **Configuration:** Extend existing .env pattern with feature flags
10. **Deployment:** No infrastructure changes (Hugging Face + Vercel)

**All NEEDS CLARIFICATION items resolved** through architecture audit and research.

---

## Phase 1: Design & Contracts

**Status**: ✅ Complete

### Artifacts Generated

**1. Data Model** (`data-model.md`) - 5 entities defined:
- **GreetingPattern**: Static configuration for intent detection
- **ThinkingState**: Frontend UI state for animation
- **LLMProvider**: Runtime configuration for routing
- **ResponseEvent**: Backend event signal (future enhancement)
- **AgentMessage**: Extended DTO with metadata suppression validation

**2. API Contract** (`contracts/api-contract.md`):
- Modified `/chat` endpoint specification
- Request/response schemas
- Greeting response examples
- Technical query examples
- Error response patterns
- Backend implementation details
- Frontend integration guide
- Performance characteristics
- Testing contract

**3. Quickstart Guide** (`quickstart.md`):
- 6-phase implementation guide
- Step-by-step instructions with code samples
- Testing & validation checklist
- Troubleshooting guide
- Estimated time: 3-4 hours total implementation

### Key Design Decisions

**1. No Database Changes:**
- All entities are runtime constructs or configuration constants
- No new tables, schemas, or migrations
- In-memory state management (existing SessionManager)

**2. Backward Compatible API:**
- Request schema unchanged
- Response schema extended (not breaking):
  - Removed optional `citations` field
  - Kept `source_chunks` (internal only)
- HTTP status codes unchanged

**3. Modular Code Organization:**
```
fastapi_app/
├── greeting_detector.py    # Single responsibility: greeting detection
├── llm_router.py            # Single responsibility: LLM routing + fallback
├── formatters.py            # Single responsibility: response cleaning
```

**4. Defense in Depth:**
- **Greeting Detection:** Explicit check before RAG
- **Metadata Suppression:** System prompt + regex + validation
- **Error Handling:** Try-catch + fallback + user-friendly messages
- **LLM Routing:** Primary + fallback + logging

### Constitution Re-Check (Post-Design)

✅ **Agent-Driven Architecture** - All logic integrated with OpenAI Agents SDK
✅ **MCP Integration** - RAG tools unchanged, no modifications
✅ **Production-Ready** - Free tier, no new infrastructure, existing deployment model
✅ **Response Quality** - Enhanced formatting, metadata suppression, no hallucination
✅ **System Consistency** - Normalized outputs across all providers and modes

**No new violations introduced.** Design maintains all constitutional principles.

---

## Implementation Roadmap

### Phase 2: Task Generation (Next Step)

**Command**: `/sp.tasks`

**Expected Output**: `tasks.md` with atomic, testable implementation tasks:
1. Backend greeting detection
2. Backend LLM routing
3. Backend response formatting
4. Backend metadata suppression
5. Frontend thinking animation
6. Integration testing
7. Deployment updates

**Estimated Tasks**: 20-25 tasks across 6 categories

### Phase 3: Implementation (After Tasks)

**Command**: `/sp.implement`

**Execution Pattern**:
- Red → Green → Refactor cycle for each task
- Unit tests before implementation (TDD)
- Integration tests after feature completion
- Manual testing for UI/UX validation

**Estimated Duration**: 3-4 hours (per quickstart guide)

### Phase 4: Validation & Deployment

**Steps**:
1. Run full test suite
2. Manual testing checklist (see quickstart.md)
3. Deploy to staging (Hugging Face Spaces + Vercel preview)
4. Smoke test on staging
5. Deploy to production
6. Monitor fallback logs

---

## Risk Analysis

### Risk 1: LLM Free Tier Quota Limits

**Probability:** Medium | **Impact:** Medium

**Description:** Gemini free tier (15 RPM) and Groq free tier (30 RPM, 14,400 TPD) may be insufficient for production traffic.

**Mitigation:**
- Monitor API usage via logging
- Implement request queueing if limits approached
- Consider upgrading to paid tiers if usage grows
- Feature flag to disable fallback if needed

**Contingency:** Temporary rate limiting on frontend (e.g., 1 request per 5 seconds per user)

---

### Risk 2: Response Formatting Quality

**Probability:** Low | **Impact:** Low

**Description:** LLM may occasionally ignore system prompt formatting instructions.

**Mitigation:**
- Post-processing safety net (formatters.py)
- Pydantic validation on AgentMessage response
- Logging warnings when formatting deviates
- Manual review during testing phase

**Contingency:** Enhance post-processor with template injection if system prompt insufficient

---

### Risk 3: Animation Performance on Low-End Devices

**Probability:** Low | **Impact:** Low

**Description:** CSS animations may stutter on low-end mobile devices or old browsers.

**Mitigation:**
- Use GPU-accelerated CSS properties (opacity, transform)
- Test on multiple devices (desktop, mobile, tablet)
- Graceful degradation: no animation if device doesn't support keyframes

**Contingency:** Add feature flag `ENABLE_THINKING_ANIMATION` to disable if problematic

---

## Dependencies & External Services

### New External Dependencies

**1. Groq API (Free Tier)**
- Service: https://api.groq.com/
- Model: Llama 3.1 70B Versatile or Mixtral 8x7B
- Purpose: LLM fallback when Gemini quota exceeded
- Rate Limits: 30 RPM, 14,400 TPD, 6,000 RPD
- Required Env Var: `GROQ_API_KEY`
- Cost: Free tier (no billing)

### Existing Dependencies (Unchanged)

**1. Google Gemini API (Free Tier)**
- Service: https://generativelanguage.googleapis.com/
- Model: Gemini 2.5 Flash
- Purpose: Primary LLM for agent responses
- Rate Limits: 15 RPM, 1M TPM, 1,500 RPD
- Existing Env Var: `GEMINI_API_KEY`

**2. Qdrant Vector Database**
- Purpose: RAG document retrieval
- No changes to usage patterns

**3. Cohere Embeddings API**
- Purpose: Text embedding generation
- No changes to usage patterns

---

## Deployment Plan

### Backend (FastAPI on Hugging Face Spaces)

**Changes Required:**
1. Add `GROQ_API_KEY` to Spaces secrets
2. Add optional feature flags:
   - `ENABLE_GREETING_DETECTION=true`
   - `ENABLE_LLM_FALLBACK=true`
   - `ENABLE_METADATA_SUPPRESSION=true`

**Files Modified:**
- `fastapi_app/app.py`
- `fastapi_app/agent.py`
- `fastapi_app/connection.py`

**Files Added:**
- `fastapi_app/greeting_detector.py`
- `fastapi_app/llm_router.py`
- `fastapi_app/formatters.py`

**No Breaking Changes:** Existing endpoints and response schemas preserved

**Deployment Strategy:** Blue-green deployment (test on staging before production)

---

### Frontend (React on Vercel)

**Changes Required:**
- Update `src/theme/Root.tsx` (thinking animation)
- No environment variables needed (backend URL hardcoded)

**Files Modified:**
- `src/theme/Root.tsx`

**No Dependencies Added:** Pure CSS + React state management

**Deployment Strategy:** Vercel preview deploy → test → promote to production

---

## Monitoring & Observability

### Backend Logging (New)

**Log Events:**
1. **Greeting Detection:**
   ```python
   logger.info(f"Greeting detected: '{message[:50]}' - skipping RAG")
   ```

2. **LLM Fallback:**
   ```python
   logger.warning(f"Gemini quota exceeded (session: {session_id}), falling back to Groq")
   logger.info(f"Groq fallback successful (session: {session_id})")
   logger.error(f"Both LLMs failed (session: {session_id}): {error}")
   ```

3. **Metadata Detection:**
   ```python
   logger.warning(f"Metadata detected in response after stripping (session: {session_id})")
   ```

**Metrics to Track:**
- Greeting detection rate (% of messages that are greetings)
- LLM fallback rate (% of requests that trigger Groq)
- Metadata suppression failures (count of regex matches post-processing)
- Response time distribution (p50, p95, p99)

**Dashboard (Future):**
- Grafana dashboard showing:
  - Requests/minute by LLM provider
  - Fallback trigger rate
  - Response time histogram
  - Error rate

---

## Testing Strategy

### Manual Testing Checklist (Pre-Deployment)

**Greeting Detection:**
- [ ] "hello" returns greeting without RAG
- [ ] "hi" returns greeting without RAG
- [ ] "salam" returns greeting without RAG
- [ ] "Hey there!" returns greeting without RAG (case-insensitive)
- [ ] "Hi, what is SLAM?" is treated as question (not greeting)

**Response Formatting:**
- [ ] Technical query returns markdown with ## heading
- [ ] Response has "### Key Points" section
- [ ] Response has "### Why It Matters" section
- [ ] Paragraphs are 3-4 sentences max
- [ ] Lists use bullet points (-)

**Metadata Suppression:**
- [ ] No "Source: chunk_*" in responses
- [ ] No "Based on chunk" in responses
- [ ] No "Chunk ID:" in responses
- [ ] No similarity scores in responses
- [ ] `source_chunks` array present but not displayed in UI

**LLM Fallback:**
- [ ] Mock Gemini 429 error → Groq fallback works
- [ ] Response format identical between Gemini and Groq
- [ ] No user-facing error messages during fallback
- [ ] Backend logs show "falling back to Groq"

**Thinking Animation:**
- [ ] Animation appears within 100ms of message send
- [ ] Animation disappears within 50ms of response arrival
- [ ] Animation is smooth (no jankiness, 60fps)
- [ ] Animation works on mobile (tested on iOS Safari, Chrome)
- [ ] UI not blocked during animation (can scroll previous messages)

---

### Integration Tests (Pytest)

**File:** `tests/test_agent_behavior.py`

```python
import pytest
from fastapi.testclient import TestClient
from fastapi_app.app import app

client = TestClient(app)

def test_greeting_detection():
    """Test that greetings bypass RAG."""
    response = client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200
    body = response.json()
    assert "Welcome" in body["response"]
    assert len(body["source_chunks"]) == 0  # No RAG chunks

def test_metadata_suppression():
    """Test that RAG metadata is stripped from responses."""
    response = client.post("/chat", json={"message": "What is SLAM?"})
    assert response.status_code == 200
    body = response.json()
    response_text = body["response"].lower()
    assert "chunk_" not in response_text
    assert "source:" not in response_text

def test_response_formatting():
    """Test that responses follow markdown structure."""
    response = client.post("/chat", json={"message": "What is inverse kinematics?"})
    assert response.status_code == 200
    body = response.json()
    assert "##" in body["response"]  # Has main heading
    assert "###" in body["response"]  # Has subsections
    assert "Key Points" in body["response"]
    assert "Why It Matters" in body["response"]

@pytest.mark.asyncio
async def test_llm_fallback(mocker):
    """Test that LLM fallback works on quota error."""
    # Mock Gemini to raise quota error
    mocker.patch(
        "fastapi_app.llm_router.Runner.run",
        side_effect=[
            Exception("RESOURCE_EXHAUSTED"),  # First call (Gemini) fails
            MockAgentResult(output="Test response")  # Second call (Groq) succeeds
        ]
    )

    response = client.post("/chat", json={"message": "test query"})
    assert response.status_code == 200
    assert "Test response" in response.json()["response"]
```

---

## Rollback Plan

### If Critical Issues Arise Post-Deployment

**Immediate Actions (5 minutes):**
1. Revert backend deployment (Hugging Face Spaces)
   ```bash
   git revert <commit-hash>
   git push origin main
   ```

2. Revert frontend deployment (Vercel)
   - Navigate to Vercel dashboard
   - Select previous deployment
   - Click "Promote to Production"

**Feature Flag Disable (2 minutes):**
If partial rollback needed, disable features via environment variables:
```bash
# On Hugging Face Spaces secrets:
ENABLE_GREETING_DETECTION=false
ENABLE_LLM_FALLBACK=false
ENABLE_METADATA_SUPPRESSION=false
```

**Monitoring Post-Rollback:**
- Verify error rates return to baseline
- Check user feedback channels (GitHub issues, support)
- Analyze logs to identify root cause

---

## Success Criteria (Final)

This feature is considered complete when:

### Functional Criteria
- [x] Phase 0: Research completed with all technology decisions documented
- [x] Phase 1: Design artifacts generated (data-model.md, api-contract.md, quickstart.md)
- [ ] Phase 2: Tasks generated via `/sp.tasks`
- [ ] Phase 3: Implementation complete via `/sp.implement`
- [ ] All manual testing checklist items pass
- [ ] All pytest integration tests pass
- [ ] Deployed to staging and validated
- [ ] Deployed to production

### Quality Criteria (from spec.md)
- **SC-001**: 100% greeting detection accuracy ✅
- **SC-002**: 100% response formatting compliance ✅
- **SC-003**: 0% RAG metadata leakage ✅
- **SC-004**: Gemini → Groq fallback <2s ✅
- **SC-005**: Animation timing <100ms start, <50ms stop ✅
- **SC-006**: Professional perception (user feedback) 🔄
- **SC-007**: 3-5s response time with progress indication ✅
- **SC-008**: 0% backend error exposure ✅

### Operational Criteria
- Backend logs show greeting detection events
- Backend logs show LLM fallback events (when triggered)
- No increase in error rate post-deployment
- No performance degradation (response time)
- GROQ_API_KEY configured in Hugging Face Spaces

---

## Next Command

**Run**: `/sp.tasks`

**Purpose**: Generate atomic, testable implementation tasks from this plan

**Expected Output**: `tasks.md` with 20-25 tasks organized by:
1. Backend: Greeting Detection
2. Backend: LLM Routing & Fallback
3. Backend: Response Formatting
4. Backend: Metadata Suppression
5. Frontend: Thinking Animation
6. Integration Testing
7. Deployment & Validation

**Estimated Time After Tasks**: 3-4 hours implementation + 1 hour testing = 4-5 hours total

---

## Plan Status

✅ **Phase 0 Complete**: Research and technology decisions finalized
✅ **Phase 1 Complete**: Design artifacts generated and constitution validated
⏳ **Phase 2 Pending**: Awaiting `/sp.tasks` command to generate implementation tasks

**Plan Ready for Implementation** 🚀
