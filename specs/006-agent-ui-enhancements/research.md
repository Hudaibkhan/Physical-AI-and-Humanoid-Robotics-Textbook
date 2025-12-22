# Research & Technology Decisions: Agent Behavior & UI Enhancement

**Feature**: 006-agent-ui-enhancements
**Date**: 2025-12-20
**Research Phase**: Phase 0

## Executive Summary

This document resolves all technical unknowns and documents technology choices for implementing the Agent Behavior & UI Enhancement feature. Based on architecture audit and requirements analysis, all major decisions have been finalized.

---

## 1. Greeting Intent Detection Approach

### Decision: Pattern Matching with Early Exit

**Rationale:**
- Simple, deterministic, and performant (<1ms execution)
- No ML model needed (keeping system lightweight)
- Exact match on predefined greeting list: `["hi", "hello", "hey", "salam", "assalam o alaikum"]`
- Case-insensitive comparison using `.lower().strip()`
- Implemented as pre-processing step before RAG execution

**Implementation Location:** `fastapi_app/agent.py` (new greeting_detector function)

**Alternatives Considered:**
1. **LLM-based intent classification** - Rejected: Adds latency (200-500ms), unnecessary for simple greetings, increases API costs
2. **Regex pattern matching** - Rejected: Overkill for exact string matching, adds complexity
3. **NLP library (spaCy/NLTK)** - Rejected: Heavy dependency for simple task, slow initialization

**Code Pattern:**
```python
GREETING_PATTERNS = {"hi", "hello", "hey", "salam", "assalam o alaikum"}

def is_greeting(message: str) -> bool:
    """Check if message is a greeting before RAG execution."""
    cleaned = message.lower().strip()
    return cleaned in GREETING_PATTERNS
```

---

## 2. LLM Routing & Fallback Architecture

### Decision: Try-Catch Wrapper with Silent Fallback

**Primary LLM:** Gemini 2.5 Flash (via Google's OpenAI-compatible API)
**Fallback LLM:** Groq (Llama 3.1 70B or Mixtral 8x7B)

**Rationale:**
- Both providers offer free tier with generous limits
- OpenAI Agents SDK supports custom LLM providers via litellm
- Silent fallback preserves user experience (no error exposure)
- Same prompt/instructions ensure output format consistency

**Implementation Location:** `fastapi_app/connection.py` (new LLMRouter class)

**Error Detection Strategy:**
```python
# Gemini quota errors to catch:
- HTTP 429 (Too Many Requests)
- RESOURCE_EXHAUSTED gRPC status
- "quota exceeded" in error message (case-insensitive)
```

**Alternatives Considered:**
1. **Load balancer approach** - Rejected: Adds complexity, both models should be tried sequentially
2. **User-selectable model** - Rejected: Violates silent fallback requirement
3. **OpenAI GPT-4 fallback** - Rejected: Not free tier, violates cost constraints

**Groq Configuration:**
```python
groq_client = AsyncOpenAI(
    api_key=os.getenv("GROQ_API_KEY"),
    base_url="https://api.groq.com/openai/v1"
)

groq_model = OpenAIChatCompletionsModel(
    model="llama-3.1-70b-versatile",  # or "mixtral-8x7b-32768"
    openai_client=groq_client
)
```

**Logging Strategy:**
- Server-side only: `logger.warning(f"Gemini quota exceeded, falling back to Groq")`
- Include: timestamp, session_id, error type
- No user-facing messages

---

## 3. Response Formatting Strategy

### Decision: System Prompt Enhancement + Post-Processing

**Two-Layer Approach:**

**Layer 1: System Prompt Instructions**
```python
instructions="""
You are a helpful robotics instructor specializing in Physical AI, ROS 2, and Humanoid Robotics.

RESPONSE STRUCTURE (MANDATORY):
## {Topic Title}

{Brief 2-3 sentence explanation}

### Key Points
- {Point 1}
- {Point 2}
- {Point 3}

### Why It Matters
- {Practical relevance}
- {Application context}

FORMATTING RULES:
- Use markdown headings (## for topic, ### for sections)
- Keep paragraphs to 3-4 sentences maximum
- Use bullet points for lists
- Write in a friendly, calm, confident tone
- Be precise and professional

CITATION REQUIREMENT:
- Base answers ONLY on provided context from RAG retrieval
- Do NOT hallucinate information
- If context insufficient, say so clearly
"""
```

**Layer 2: Post-Processing (Optional Safety Net)**
- Applied only if LLM ignores formatting instructions
- Checks for markdown headings presence
- Adds minimal structure if missing
- Preserves content, only fixes format

**Rationale:**
- Modern LLMs (Gemini 2.5 Flash, Groq models) follow structured prompts reliably
- Post-processing as safety net prevents edge case failures
- Maintains control over output quality

**Implementation Location:**
- System prompt: `fastapi_app/agent.py` (update book_rag_agent instructions)
- Post-processor: `fastapi_app/formatters.py` (new module)

**Alternatives Considered:**
1. **Template-based generation** - Rejected: Loses LLM's natural language quality
2. **Pure post-processing** - Rejected: Harder to maintain, reactive instead of proactive
3. **JSON mode output** - Rejected: Loses markdown formatting, requires additional conversion

---

## 4. RAG Metadata Suppression

### Decision: Response Filter Pipeline

**Three-Stage Suppression:**

**Stage 1: Agent Instructions (Primary)**
```python
"Do NOT include:
- Chunk IDs or references like 'Based on chunk X'
- Source metadata or document identifiers
- Internal retrieval details
- RAG system information

Write responses as if you authored them directly."
```

**Stage 2: Response Cleaning (Secondary)**
```python
def strip_rag_metadata(text: str) -> str:
    """Remove RAG metadata from agent responses."""
    # Remove patterns like:
    # - "Source: chunk_123"
    # - "Based on chunk X"
    # - "According to document Y"
    # - "[Retrieved from: ...]"

    patterns = [
        r'Source:\s*chunk_\w+',
        r'Based on chunk \w+',
        r'According to document \w+',
        r'\[Retrieved from:.*?\]',
        r'Chunk ID:.*',
        r'Similarity score:.*',
    ]

    for pattern in patterns:
        text = re.sub(pattern, '', text, flags=re.IGNORECASE)

    return text.strip()
```

**Stage 3: Backend Response Model (Tertiary)**
- Keep `source_chunks` in AgentResponse for debugging/audit
- Frontend receives chunks but doesn't display them to users
- Citations field removed from user-facing display

**Rationale:**
- Defense in depth: Multiple layers ensure metadata never leaks
- System prompt most effective (prevents generation)
- Regex cleaning catches edge cases
- Backend model separation preserves audit trail

**Implementation Location:**
- System prompt: `fastapi_app/agent.py`
- Cleaning function: `fastapi_app/formatters.py`
- Response handling: `fastapi_app/app.py` (modify response construction)

**Alternatives Considered:**
1. **Post-only filtering** - Rejected: Reactive, metadata still generated
2. **Remove source_chunks entirely** - Rejected: Loses debugging capability
3. **Frontend-side filtering** - Rejected: Metadata already sent over network, inefficient

---

## 5. Frontend Thinking Animation Design

### Decision: CSS Keyframe Animation with React State

**Animation Specification:**
- **Type:** Three-dot wave animation
- **Duration:** 1 second loop
- **Effect:** Opacity fade (0.3 → 1.0 → 0.3)
- **Stagger:** 150ms delay between dots
- **Colors:** Neutral gray (#94a3b8 - tailwind slate-400)

**CSS Implementation:**
```css
@keyframes wave {
  0%, 100% { opacity: 0.3; }
  50% { opacity: 1; }
}

.thinking-dot {
  display: inline-block;
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: #94a3b8;
  margin: 0 4px;
  animation: wave 1s ease-in-out infinite;
}

.thinking-dot:nth-child(1) { animation-delay: 0ms; }
.thinking-dot:nth-child(2) { animation-delay: 150ms; }
.thinking-dot:nth-child(3) { animation-delay: 300ms; }
```

**React State Management:**
```typescript
const [isThinking, setIsThinking] = useState(false);

// On send:
setIsThinking(true);

// On first token or complete response:
setIsThinking(false);
```

**Rationale:**
- Pure CSS animations perform at 60fps (GPU-accelerated)
- No JavaScript libraries needed (meets zero-dependency constraint)
- Simple state toggle (boolean, not complex state machine)
- Accessible (works with screen readers, no flashing/seizure risk)

**Implementation Location:** `src/theme/Root.tsx`

**Alternatives Considered:**
1. **Lottie animation** - Rejected: Adds 20KB+ dependency, overkill for simple effect
2. **SVG animation** - Rejected: More complex DOM structure, same effect achievable with CSS
3. **React Spring** - Rejected: Paid/complex dependency, violates constraints
4. **Canvas animation** - Rejected: Poor performance, accessibility issues

---

## 6. Backend Response Streaming Implementation

### Decision: Deferred (Not Required for MVP)

**Current Architecture Analysis:**
- Existing `/chat` endpoint returns complete JSON response
- OpenAI Agents SDK supports streaming via `Runner.stream()`
- Frontend uses `fetch()` with single `response.json()` call

**Streaming Readiness:**
```python
# Backend capability exists but not implemented:
async for chunk in Runner.stream(book_rag_agent, input=message):
    yield chunk  # Would require SSE or WebSocket

# Frontend would need:
# - EventSource for SSE, or
# - WebSocket connection, or
# - fetch() with ReadableStream
```

**Decision: Implement "Response Started" Signal Without Full Streaming**

**Approach:**
1. Keep existing non-streaming response
2. Add immediate acknowledgment response before processing
3. Frontend detects acknowledgment → stops thinking animation
4. Full response arrives shortly after (3-5s typical)

**Implementation Pattern:**
```python
# Backend sends immediate ACK:
@app.post("/chat")
async def chat(request: AgentRequest):
    # Option 1: Background task pattern
    task_id = str(uuid.uuid4())
    background_tasks.add_task(process_agent_request, task_id, request)
    return {"status": "processing", "task_id": task_id}

    # Frontend polls or waits for SSE event
```

**Alternative: Simpler Approach (Recommended)**
- Keep current synchronous flow
- Accept 3-5s latency
- Thinking animation runs full duration
- No partial streaming needed

**Rationale:**
- True streaming adds significant complexity (SSE/WebSocket infrastructure)
- Current 3-5s response time acceptable with thinking animation
- Can be added later if user feedback demands it
- Focus on correctness over premature optimization

**Future Consideration:**
If response times exceed 10s, implement:
- Server-Sent Events (SSE) for token streaming
- `Runner.stream()` on backend
- `EventSource` on frontend
- Progressive message rendering

---

## 7. Error Handling Architecture

### Decision: Graceful Degradation with User-Friendly Messages

**Error Categories & Handling:**

**1. LLM API Errors (Gemini/Groq)**
```python
try:
    result = await Runner.run(book_rag_agent, input=message, config=gemini_config)
except QuotaExceededError:
    # Silent fallback to Groq (no user message)
    result = await Runner.run(book_rag_agent, input=message, config=groq_config)
except NetworkError as e:
    return AgentResponse(
        response="I'm having trouble connecting to my knowledge base. Please try again in a moment.",
        session_id=session_id
    )
```

**2. RAG Retrieval Errors (Qdrant)**
```python
try:
    chunks = await connection_manager.direct_rag_search(query)
except QdrantException as e:
    logger.error(f"Qdrant error: {e}")
    # Fallback: Answer without RAG (general knowledge only)
    return AgentResponse(
        response="I can provide a general answer, but my textbook search is temporarily unavailable...",
        session_id=session_id
    )
```

**3. Greeting Detection Errors**
```python
# No errors possible (simple string matching)
# If detection fails, worst case: greeting gets RAG treatment (acceptable)
```

**4. Frontend Network Errors**
```typescript
try {
    const response = await fetch('/chat', {...});
    if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
    }
} catch (error) {
    setMessages(prev => [...prev, {
        role: 'assistant',
        content: '❌ Connection error. Please check your internet connection and try again.',
        timestamp: new Date()
    }]);
    setIsThinking(false);  // Stop animation
}
```

**Rationale:**
- Never expose stack traces, API keys, or internal paths
- Provide actionable guidance to users
- Log full errors server-side for debugging
- Maintain service continuity with fallbacks

**Implementation Locations:**
- Backend errors: `fastapi_app/app.py` (try-catch in /chat endpoint)
- LLM routing: `fastapi_app/connection.py` (LLMRouter class)
- Frontend errors: `src/theme/Root.tsx` (catch blocks in sendMessage)

---

## 8. Testing Strategy

### Decision: Manual Testing + Integration Tests

**Test Scenarios (Manual Execution):**

**1. Greeting Detection**
```
Input: "hi"
Expected: Friendly greeting, no RAG call, <100ms response
Verification: Check backend logs for "Greeting detected, skipping RAG"

Input: "hello there"
Expected: Same behavior

Input: "Hi, what is SLAM?"
Expected: Greeting + question (treat as question, enable RAG)
```

**2. Response Formatting**
```
Input: "What is inverse kinematics?"
Expected: Markdown structure (##, ###, bullets), 3-4 sentence paragraphs
Verification: Visual inspection + regex check for markdown

Input: "Explain ROS 2"
Expected: Same formatting
```

**3. Metadata Suppression**
```
Input: "What is SLAM?"
Expected: No "Source:", "chunk_", "Based on", references in response
Verification: Regex search for forbidden patterns
```

**4. LLM Fallback**
```
Setup: Mock Gemini 429 error
Input: Any question
Expected: Response delivered via Groq, identical format, no error message
Verification: Backend logs show "Fallback activated"
```

**5. Thinking Animation**
```
Action: Send message
Expected: Animation appears within 100ms
Action: Response arrives
Expected: Animation disappears within 50ms
Verification: Browser DevTools performance timeline
```

**Integration Tests (Pytest):**
```python
# tests/test_agent_behavior.py
async def test_greeting_detection():
    response = await client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200
    assert "RAG" not in response.json()["response"]

async def test_metadata_suppression():
    response = await client.post("/chat", json={"message": "What is SLAM?"})
    body = response.json()["response"]
    assert "chunk_" not in body.lower()
    assert "source:" not in body.lower()

async def test_llm_fallback(mock_gemini_error):
    response = await client.post("/chat", json={"message": "test"})
    assert response.status_code == 200
    assert mock_groq_called  # Verify fallback triggered
```

**Rationale:**
- Manual testing sufficient for UX validation (animations, formatting)
- Integration tests catch regressions in core logic
- No unit tests for simple functions (greeting detection is trivial)
- Focus on end-to-end behavior verification

---

## 9. Environment Variables & Configuration

### Decision: Extend Existing .env Pattern

**New Variables Required:**

```bash
# LLM Configuration
GEMINI_API_KEY=<existing>
GROQ_API_KEY=<new-required>

# RAG Configuration (existing)
QDRANT_URL=<existing>
QDRANT_API_KEY=<existing>
COHERE_API_KEY=<existing>
COLLECTION_NAME=book_chunks

# Feature Flags (new-optional)
ENABLE_GREETING_DETECTION=true
ENABLE_LLM_FALLBACK=true
ENABLE_METADATA_SUPPRESSION=true

# Logging (new-optional)
LOG_LEVEL=INFO
LOG_LLM_FALLBACKS=true
```

**Configuration Loading:**
```python
# fastapi_app/config.py (new file)
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    gemini_api_key: str
    groq_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    cohere_api_key: str
    collection_name: str = "book_chunks"
    enable_greeting_detection: bool = True
    enable_llm_fallback: bool = True
    enable_metadata_suppression: bool = True
    log_level: str = "INFO"

    class Config:
        env_file = ".env"

settings = Settings()
```

**Rationale:**
- Pydantic Settings for type safety and validation
- Feature flags enable gradual rollout and A/B testing
- Separate config module improves maintainability
- Follows existing project patterns

---

## 10. Performance Considerations

### Expected Latencies

| Operation | Target | Measurement |
|-----------|--------|-------------|
| Greeting detection | <1ms | String comparison |
| RAG retrieval (Qdrant) | 50-200ms | Network + vector search |
| LLM inference (Gemini) | 2-4s | API call + generation |
| LLM fallback (Groq) | +1-2s | Additional API call |
| Total response time | 3-5s | End-to-end |
| Animation start | <100ms | React state update |
| Animation stop | <50ms | React state update |

### Optimization Opportunities (Future)

1. **Greeting Cache:** Precompute responses for common greetings
2. **RAG Caching:** Cache frequent queries for 5 minutes
3. **Streaming:** Implement token streaming for faster perceived response
4. **Parallel RAG:** Run RAG retrieval concurrently with greeting check
5. **Connection Pooling:** Reuse HTTP connections to APIs

**Current Decision:** No premature optimization. Current latency acceptable with thinking animation.

---

## 11. Deployment Considerations

### Backend (FastAPI)

**Current Deployment:** Hugging Face Spaces

**Changes Required:**
- Add `GROQ_API_KEY` to Spaces secrets
- Update `requirements.txt` (no new dependencies)
- Verify free tier limits:
  - Gemini: 15 RPM (requests per minute), 1M TPM (tokens per minute)
  - Groq: 30 RPM, 14,400 TPD (tokens per day) for Llama 3.1 70B

**Deployment Checklist:**
- [ ] Environment variables configured
- [ ] Health check endpoint (`/health`) added
- [ ] Logging configured (no secrets in logs)
- [ ] Rate limiting considered (if needed)

### Frontend (Docusaurus + Vercel)

**Current Deployment:** Vercel

**Changes Required:**
- Update `src/theme/Root.tsx` (thinking animation)
- No new dependencies
- No environment variables needed (backend URL hardcoded)

**Deployment Checklist:**
- [ ] CSS animations tested in production build
- [ ] CORS configured correctly on backend
- [ ] Error messages user-friendly
- [ ] Mobile responsiveness verified

---

## 12. Rollback Plan

### If Issues Arise Post-Deployment

**Immediate Actions:**
1. **Backend:** Revert to previous commit via git
2. **Frontend:** Revert Vercel deployment to previous release
3. **Feature Flags:** Disable features via environment variables without code changes

**Feature Flag Disabling:**
```python
if not settings.enable_greeting_detection:
    # Skip greeting detection, always run RAG
    pass

if not settings.enable_llm_fallback:
    # Gemini-only, return error if fails
    pass
```

**Monitoring:**
- Backend logs: Monitor for increased error rates
- Frontend: User feedback via GitHub issues
- API limits: Track Gemini/Groq quota usage

---

## 13. Compliance with Constitution

### Constitution Alignment Check

**From `.specify/memory/constitution.md`:**

| Principle | Compliance | Notes |
|-----------|-----------|-------|
| Agent-Driven Architecture | ✅ Yes | All logic runs through OpenAI Agents SDK |
| MCP Integration | ✅ Yes | Existing RAG tools preserved, no changes |
| Production-Ready | ✅ Yes | Free tier APIs, no breaking changes |
| Agent Response Quality | ✅ Yes | Enhanced formatting, metadata suppression |
| System Consistency | ✅ Yes | Identical formatting across Gemini/Groq |

**No Constitution Violations:** All changes extend existing architecture without introducing complexity or violating principles.

---

## 14. Summary of Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Greeting Detection | Pattern matching (pre-RAG) | Simple, fast, deterministic |
| LLM Routing | Gemini → Groq silent fallback | Free tier, reliable, user-friendly |
| Response Formatting | System prompt + post-processing | LLM-first with safety net |
| Metadata Suppression | Three-stage filter | Defense in depth |
| Thinking Animation | CSS keyframe + React state | Zero dependencies, 60fps |
| Streaming | Deferred (not MVP) | Acceptable latency with animation |
| Error Handling | Graceful degradation | User-friendly messages, full logs |
| Testing | Manual + integration tests | Sufficient coverage for scope |
| Configuration | Extend .env + feature flags | Flexible, gradual rollout |
| Deployment | No infrastructure changes | Drop-in updates |

---

## 15. Open Questions (Resolved)

All unknowns from Technical Context section have been resolved through architecture audit and research. No remaining NEEDS CLARIFICATION markers.

**Phase 0 Complete** ✅

Ready to proceed to Phase 1: Design & Contracts.
