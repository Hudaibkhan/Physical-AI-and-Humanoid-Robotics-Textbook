# API Contract: Agent Behavior & UI Enhancement

**Feature**: 006-agent-ui-enhancements
**Date**: 2025-12-20
**Phase**: Phase 1 - Design

## Overview

This document specifies the API contract changes for the Agent Behavior & UI Enhancement feature. The existing `/chat` endpoint is modified to support new behaviors (greeting detection, metadata suppression, LLM fallback) while maintaining backward compatibility.

---

## Endpoint Modifications

### POST /chat

**Purpose:** Send message to agent and receive formatted response

**Existing Behavior:** Execute RAG search → Run agent → Return response with citations
**New Behavior:** Check greeting → Execute RAG (if not greeting) → Try Gemini → Fallback to Groq (if needed) → Strip metadata → Return formatted response

**URL:** `POST http://localhost:8000/chat`

**Request Headers:**
```http
Content-Type: application/json
```

**Request Body** (unchanged):
```json
{
  "message": "string (required)",
  "selected_text": "string | null (optional)",
  "session_id": "string | null (optional)"
}
```

**Request Schema:**
```typescript
interface AgentRequest {
  message: string;           // User's question or message
  selected_text?: string;    // Optional selected text from page
  session_id?: string;       // Optional session UUID for continuity
}
```

**Request Validation:**
- `message` must not be empty (min length: 1)
- `message` max length: 10,000 characters
- `selected_text` max length: 10,000 characters (if provided)
- `session_id` must be valid UUID format (if provided)

---

## Response Format (Modified)

### Success Response (200 OK)

**Response Body:**
```json
{
  "response": "string (markdown formatted)",
  "session_id": "string (UUID)",
  "source_chunks": []
}
```

**Response Schema:**
```typescript
interface AgentResponse {
  response: string;              // Formatted markdown answer (no metadata)
  session_id: string;            // Session UUID for tracking
  source_chunks: QueryResult[];  // Retrieved chunks (internal, not displayed)
}

interface QueryResult {
  id: string;                    // Chunk identifier
  content: string;               // Chunk text content
  similarity_score: number;      // Cosine similarity (0-1)
  metadata: Record<string, any>; // Additional metadata
}
```

**Changes from Existing:**
1. **Removed field:** `citations: string[]` - No longer exposed to prevent metadata leakage
2. **Modified field:** `source_chunks` - Still returned but marked as internal (frontend should not display)
3. **Enhanced field:** `response` - Now guaranteed to be markdown-formatted without RAG metadata

**Response Guarantees:**
- `response` will NEVER contain:
  - "Source: chunk_*"
  - "Based on chunk X"
  - "Chunk ID: *"
  - "Similarity score: *"
  - "Retrieved from: *"
- `response` will ALWAYS follow markdown structure:
  ```markdown
  ## {Topic}

  {Brief explanation}

  ### Key Points
  - {Point 1}
  - {Point 2}

  ### Why It Matters
  - {Relevance}
  ```
- `session_id` will always be returned (generated if not provided)
- `source_chunks` may be empty for greetings or errors

---

### Greeting Response Example

**Request:**
```json
{
  "message": "hello"
}
```

**Response:**
```json
{
  "response": "## Welcome! 👋\n\nHello! I'm your Physical AI and Humanoid Robotics textbook assistant. I'm here to help you learn about robotics, ROS 2, and humanoid systems.\n\n### How Can I Help?\n- Explain robotics concepts from the textbook\n- Answer questions about ROS 2 and navigation\n- Clarify topics related to physical AI\n\n### Why I'm Here\n- Provide instant access to course content\n- Support your learning journey with accurate information\n\nFeel free to ask me anything about the course materials!",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "source_chunks": []
}
```

**Note:** `source_chunks` is empty because greeting does not trigger RAG retrieval.

---

### Technical Query Response Example

**Request:**
```json
{
  "message": "What is SLAM?"
}
```

**Response:**
```json
{
  "response": "## Simultaneous Localization and Mapping (SLAM)\n\nSLAM is a computational problem in robotics where a robot builds a map of an unknown environment while simultaneously tracking its own location within that map. It's fundamental to autonomous navigation.\n\n### Key Points\n- Enables robots to navigate without pre-existing maps\n- Combines sensor data (lidar, cameras) with motion estimates\n- Solves the chicken-and-egg problem: you need a map to localize, but need localization to build a map\n- Used in self-driving cars, drones, and mobile robots\n\n### Why It Matters\n- Essential for autonomous robots in real-world environments\n- Enables exploration of unknown spaces\n- Forms the basis for advanced navigation in ROS 2\n- Critical skill for robotics engineers working on mobile platforms",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "source_chunks": [
    {
      "id": "chunk_12345",
      "content": "SLAM (Simultaneous Localization and Mapping) is a method...",
      "similarity_score": 0.92,
      "metadata": {"book_id": "ros2-navigation", "chunk_index": 42}
    },
    {
      "id": "chunk_12346",
      "content": "In ROS 2, SLAM is implemented using the slam_toolbox package...",
      "similarity_score": 0.87,
      "metadata": {"book_id": "ros2-navigation", "chunk_index": 43}
    }
  ]
}
```

**Note:**
- `response` field contains NO references to chunk IDs or sources
- `source_chunks` are returned but should NOT be displayed to users (for audit/debugging only)

---

## Error Responses

### 400 Bad Request - Invalid Input

**Scenario:** Message is empty or invalid format

**Response:**
```json
{
  "detail": "Message cannot be empty"
}
```

**Status Code:** 400

---

### 500 Internal Server Error - Both LLMs Failed

**Scenario:** Gemini quota exceeded AND Groq also failed

**Response:**
```json
{
  "response": "I'm experiencing technical difficulties connecting to my knowledge base. Please try again in a few moments. If the problem persists, check the service status.",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "source_chunks": []
}
```

**Status Code:** 200 (graceful degradation, not a hard error)

**Note:** System returns 200 OK with user-friendly message instead of exposing internal error (500). Server logs contain full error details.

---

### 503 Service Unavailable - RAG Retrieval Failed

**Scenario:** Qdrant vector database unreachable

**Response:**
```json
{
  "response": "## Temporary Service Interruption\n\nI'm unable to access the textbook content right now. I can still provide general robotics knowledge, but I can't cite specific sections from your course materials.\n\n### What You Can Do\n- Try your question again in a moment\n- Check your internet connection\n- If the issue persists, contact support\n\nI apologize for the inconvenience!",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "source_chunks": []
}
```

**Status Code:** 200 (graceful degradation)

---

## Backend Implementation Details

### Greeting Detection Logic

**Location:** `fastapi_app/agent.py` (new function)

```python
GREETING_PATTERNS = {"hi", "hello", "hey", "salam", "assalam o alaikum"}

def is_greeting(message: str) -> bool:
    """Check if message is a greeting."""
    cleaned = message.lower().strip()
    return cleaned in GREETING_PATTERNS

async def generate_greeting_response() -> str:
    """Generate friendly greeting response."""
    return """## Welcome! 👋

Hello! I'm your Physical AI and Humanoid Robotics textbook assistant...
"""
```

**Execution Point:**
```python
@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    # NEW: Check for greeting BEFORE RAG
    if is_greeting(request.message):
        return AgentResponse(
            response=await generate_greeting_response(),
            session_id=request.session_id or str(uuid.uuid4()),
            source_chunks=[]
        )

    # Existing: RAG + agent execution
    ...
```

---

### LLM Fallback Logic

**Location:** `fastapi_app/connection.py` (new LLMRouter class)

```python
class LLMRouter:
    """Manages LLM selection and fallback."""

    def __init__(self, primary_config, fallback_config):
        self.primary = primary_config
        self.fallback = fallback_config

    async def run_with_fallback(self, agent, input_message):
        """Try primary LLM, fallback to secondary on error."""
        try:
            # Try Gemini
            result = await Runner.run(agent, input=input_message, run_config=self.primary)
            return result
        except (QuotaExceededError, ResourceExhaustedError) as e:
            logger.warning(f"Primary LLM quota exceeded, falling back: {e}")
            # Silent fallback to Groq
            result = await Runner.run(agent, input=input_message, run_config=self.fallback)
            return result
```

**Execution Point:**
```python
@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    # ... after RAG retrieval ...

    # NEW: Use LLMRouter instead of direct Runner.run
    llm_router = LLMRouter(gemini_config, groq_config)
    result = await llm_router.run_with_fallback(book_rag_agent, request.message)

    # ... format and return response ...
```

---

### Metadata Suppression Logic

**Location:** `fastapi_app/formatters.py` (new module)

```python
import re

def strip_rag_metadata(text: str) -> str:
    """Remove RAG metadata from response."""
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

**Execution Point:**
```python
@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    # ... after agent execution ...

    # NEW: Strip metadata from response
    clean_response = strip_rag_metadata(result.final_output)

    return AgentResponse(
        response=clean_response,
        session_id=session_id,
        source_chunks=formatted_chunks  # Internal only, not displayed
    )
```

---

## Frontend Integration

### Request Sending (Unchanged)

**Location:** `src/theme/Root.tsx`

```typescript
const sendMessage = async (question: string, isFromSelectedText = false) => {
    setIsThinking(true);  // NEW: Start thinking animation

    const apiUrl = 'http://localhost:8000/chat';
    const requestBody = {
        message: question,
        selected_text: isFromSelectedText ? selectedText : null,
        session_id: null
    };

    const response = await fetch(apiUrl, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody),
    });

    const data = await response.json();

    setIsThinking(false);  // NEW: Stop thinking animation

    setMessages((prev) => [
        ...prev,
        {
            role: 'assistant',
            content: data.response,  // Display response (metadata-free)
            // REMOVED: sources: data.source_chunks (no longer displayed)
            timestamp: new Date(),
        },
    ]);
};
```

---

### Response Handling Changes

**Old Behavior:**
```typescript
// Display citations and source chunks
{message.sources?.map((source, i) => (
    <div key={i}>Source: {source.id}</div>
))}
```

**New Behavior:**
```typescript
// No source display - only formatted response
{message.content}  // Renders markdown with formatting
```

---

## Backward Compatibility

### Breaking Changes: None

**Existing clients will continue to work** because:
1. Request schema unchanged
2. Response schema extended (not breaking):
   - `citations` removed but was optional
   - `source_chunks` still present
3. HTTP status codes unchanged (200, 400, 500)

### Migration Path for Existing Frontends

**If frontend currently displays citations:**
```typescript
// Old code (will break):
{response.citations.map(c => <div>{c}</div>)}

// Migration (remove citation display):
// Simply remove the citation rendering code

// New code (display only formatted response):
<ReactMarkdown>{response.response}</ReactMarkdown>
```

**If frontend currently displays source_chunks:**
```typescript
// Old code (still works but should be removed):
{response.source_chunks.map(chunk => <div>{chunk.content}</div>)}

// New code (remove source display):
// Simply remove the source chunk rendering code
```

---

## Performance Characteristics

### Latency Budget

| Operation | Target | Notes |
|-----------|--------|-------|
| Greeting detection | <1ms | String comparison |
| RAG retrieval | 50-200ms | Qdrant search |
| Gemini inference | 2-4s | LLM generation |
| Groq fallback | +1-2s | Only on Gemini failure |
| Metadata stripping | <5ms | Regex processing |
| **Total (greeting)** | **<500ms** | No RAG, fast path |
| **Total (query, no fallback)** | **3-5s** | Standard path |
| **Total (query, with fallback)** | **5-7s** | Fallback path |

### Rate Limits

**Gemini 2.5 Flash (Free Tier):**
- 15 requests per minute (RPM)
- 1,000,000 tokens per minute (TPM)
- 1,500 requests per day (RPD)

**Groq Llama 3.1 70B (Free Tier):**
- 30 requests per minute (RPM)
- 14,400 tokens per day (TPD)
- 6,000 requests per day (RPD)

**Recommendation:** Implement request queueing if limits approached.

---

## Testing Contract

### Test Scenarios

**1. Greeting Detection:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "hello"}'

# Expected:
# - response contains "Welcome"
# - source_chunks is empty array
# - session_id is UUID
# - NO "chunk_" or "Source:" in response
```

**2. Technical Query:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is SLAM?"}'

# Expected:
# - response contains markdown headers (##, ###)
# - response has "Key Points" and "Why It Matters" sections
# - source_chunks has entries (but not displayed in UI)
# - NO metadata in response text
```

**3. LLM Fallback (mock Gemini error):**
```bash
# Requires setting mock error in backend
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test query"}'

# Expected:
# - Response still 200 OK
# - Response formatted identically
# - Backend logs show "Falling back to Groq"
# - NO user-facing error message
```

---

## Summary of Changes

| Aspect | Old Behavior | New Behavior |
|--------|-------------|--------------|
| Greeting handling | Always triggers RAG | Detected early, skips RAG |
| Response format | Inconsistent, sometimes plain text | Always markdown-structured |
| Metadata exposure | Citations and chunks visible | Metadata stripped, internal only |
| LLM selection | Gemini only | Gemini → Groq silent fallback |
| Error handling | Exposes internal errors | Graceful degradation |
| Thinking state | No indication | Animation on frontend |

**Contract Status:** ✅ Fully specified and backward-compatible
