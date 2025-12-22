# Quickstart: Agent Behavior & UI Enhancement

**Feature**: 006-agent-ui-enhancements
**Target Audience**: Developers implementing this feature
**Estimated Reading Time**: 10 minutes

## Overview

This guide provides a step-by-step implementation path for the Agent Behavior & UI Enhancement feature. Follow these instructions to add greeting detection, LLM fallback, response formatting, metadata suppression, and thinking animations.

---

## Prerequisites

Before starting, ensure you have:
- [x] Existing FastAPI backend running (`fastapi_app/`)
- [x] OpenAI Agents SDK integrated (`agents` library v0.0.14+)
- [x] Gemini API key configured (`GEMINI_API_KEY` in `.env`)
- [x] Qdrant vector database accessible
- [x] React frontend running (`src/theme/Root.tsx`)
- [x] Git branch checked out: `006-agent-ui-enhancements`

---

## Implementation Phases

### Phase 1: Backend - Greeting Detection (30 min)

**Goal:** Detect greetings before RAG execution and return friendly responses.

**Step 1.1: Create greeting detector**

Create `fastapi_app/greeting_detector.py`:

```python
"""Greeting detection module."""

from typing import Set

class GreetingDetector:
    """Detects greeting messages to bypass RAG."""

    GREETING_PATTERNS: Set[str] = {
        "hi",
        "hello",
        "hey",
        "salam",
        "assalam o alaikum"
    }

    @classmethod
    def is_greeting(cls, message: str) -> bool:
        """
        Check if message is a greeting.

        Args:
            message: User message to check

        Returns:
            True if greeting, False otherwise
        """
        cleaned = message.lower().strip()
        return cleaned in cls.GREETING_PATTERNS

    @classmethod
    def generate_greeting_response(cls) -> str:
        """Generate friendly greeting response."""
        return """## Welcome! 👋

Hello! I'm your Physical AI and Humanoid Robotics textbook assistant. I'm here to help you learn about robotics, ROS 2, and humanoid systems.

### How Can I Help?
- Explain robotics concepts from the textbook
- Answer questions about ROS 2 and navigation
- Clarify topics related to physical AI

### Why I'm Here
- Provide instant access to course content
- Support your learning journey with accurate information

Feel free to ask me anything about the course materials!"""
```

**Step 1.2: Integrate into /chat endpoint**

Edit `fastapi_app/app.py` (line ~350, before RAG execution):

```python
from greeting_detector import GreetingDetector  # Add at top

@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    session_id = request.session_id or str(uuid.uuid4())

    # NEW: Check for greeting BEFORE RAG
    if GreetingDetector.is_greeting(request.message):
        return AgentResponse(
            response=GreetingDetector.generate_greeting_response(),
            session_id=session_id,
            source_chunks=[]
        )

    # Existing RAG + agent logic continues...
```

**Test:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "hello"}'

# Expected: Greeting response without RAG chunks
```

---

### Phase 2: Backend - LLM Fallback (45 min)

**Goal:** Silent fallback from Gemini to Groq on quota errors.

**Step 2.1: Get Groq API key**

1. Visit https://console.groq.com/
2. Create account and generate API key
3. Add to `.env`:
   ```bash
   GROQ_API_KEY=gsk_...
   ```

**Step 2.2: Create LLM router**

Create `fastapi_app/llm_router.py`:

```python
"""LLM routing and fallback logic."""

import logging
from typing import Optional
from openai import AsyncOpenAI
from agents import Runner
from agents.extensions.models.openai_chat_completions_model import OpenAIChatCompletionsModel

logger = logging.getLogger(__name__)

class QuotaExceededError(Exception):
    """Raised when LLM quota is exceeded."""
    pass

class LLMRouter:
    """Manages LLM selection and fallback."""

    def __init__(self, gemini_config, groq_config):
        """
        Initialize router with primary and fallback configs.

        Args:
            gemini_config: RunConfig for Gemini
            groq_config: RunConfig for Groq
        """
        self.gemini_config = gemini_config
        self.groq_config = groq_config
        self.fallback_count = 0

    async def run_with_fallback(self, agent, input_message):
        """
        Run agent with primary LLM, fallback on quota error.

        Args:
            agent: OpenAI agent instance
            input_message: User message

        Returns:
            Agent execution result

        Raises:
            Exception: If both LLMs fail
        """
        try:
            # Try Gemini (primary)
            result = await Runner.run(
                agent,
                input=input_message,
                run_config=self.gemini_config
            )
            return result

        except Exception as e:
            error_msg = str(e).lower()

            # Check if quota/rate limit error
            if any(keyword in error_msg for keyword in ['quota', '429', 'resource_exhausted', 'rate limit']):
                logger.warning(f"Gemini quota exceeded, falling back to Groq: {e}")
                self.fallback_count += 1

                # Silent fallback to Groq
                try:
                    result = await Runner.run(
                        agent,
                        input=input_message,
                        run_config=self.groq_config
                    )
                    logger.info("Groq fallback successful")
                    return result

                except Exception as groq_error:
                    logger.error(f"Groq fallback also failed: {groq_error}")
                    raise Exception("Both LLMs unavailable") from groq_error
            else:
                # Not a quota error, re-raise
                raise
```

**Step 2.3: Configure Groq client**

Edit `fastapi_app/connection.py` (add after Gemini config):

```python
import os
from openai import AsyncOpenAI
from agents.extensions.models.openai_chat_completions_model import OpenAIChatCompletionsModel

# Existing Gemini config...

# NEW: Groq configuration
groq_api_key = os.getenv("GROQ_API_KEY")

if groq_api_key:
    groq_client = AsyncOpenAI(
        api_key=groq_api_key,
        base_url="https://api.groq.com/openai/v1"
    )

    groq_model = OpenAIChatCompletionsModel(
        model="llama-3.1-70b-versatile",
        openai_client=groq_client
    )

    groq_config = RunConfig(
        model=groq_model,
        model_provider=groq_client,
        tracing_disabled=True
    )
else:
    groq_config = None
    logging.warning("GROQ_API_KEY not found, fallback disabled")
```

**Step 2.4: Use LLMRouter in /chat endpoint**

Edit `fastapi_app/app.py`:

```python
from llm_router import LLMRouter  # Add at top

@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    # ... after greeting check and RAG retrieval ...

    # NEW: Use LLMRouter instead of direct Runner.run
    llm_router = LLMRouter(config, groq_config)
    result = await llm_router.run_with_fallback(book_rag_agent, request.message)

    # ... rest of response handling ...
```

**Test:**
```python
# In app.py, temporarily add before agent execution:
raise Exception("RESOURCE_EXHAUSTED")  # Mock Gemini error

# Then run:
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is SLAM?"}'

# Expected: Response still works, logs show "falling back to Groq"
```

---

### Phase 3: Backend - Response Formatting (30 min)

**Goal:** Enforce structured markdown output via system prompt.

**Step 3.1: Update agent system prompt**

Edit `fastapi_app/agent.py` (book_rag_agent definition):

```python
book_rag_agent = Agent(
    name="book_rag_agent",
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

CONTENT RULES:
- Base answers ONLY on provided context from RAG retrieval
- Do NOT hallucinate information
- If context insufficient, say so clearly
- Do NOT include chunk IDs, source references, or metadata

PROHIBITED in responses:
- "Source: chunk_*"
- "Based on chunk X"
- "Chunk ID: *"
- "Retrieved from: *"

Write responses as if you authored them directly.
    """,
    model=model,
    tools=[rag_query]
)
```

**Test:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is inverse kinematics?"}'

# Expected: Response has ## heading, ### Key Points, ### Why It Matters
```

---

### Phase 4: Backend - Metadata Suppression (30 min)

**Goal:** Strip RAG metadata from responses as safety net.

**Step 4.1: Create metadata stripper**

Create `fastapi_app/formatters.py`:

```python
"""Response formatting and cleaning utilities."""

import re
from typing import List

class ResponseFormatter:
    """Cleans and formats agent responses."""

    # Patterns that indicate RAG metadata leakage
    METADATA_PATTERNS: List[str] = [
        r'Source:\s*chunk_\w+',
        r'Based on chunk \w+',
        r'According to document \w+',
        r'\[Retrieved from:.*?\]',
        r'Chunk ID:.*',
        r'Similarity score:.*',
        r'chunk_\d+',
        r'document_\d+',
    ]

    @classmethod
    def strip_metadata(cls, text: str) -> str:
        """
        Remove RAG metadata from response text.

        Args:
            text: Response text possibly containing metadata

        Returns:
            Cleaned text without metadata references
        """
        cleaned = text

        for pattern in cls.METADATA_PATTERNS:
            cleaned = re.sub(pattern, '', cleaned, flags=re.IGNORECASE)

        # Remove excessive whitespace
        cleaned = re.sub(r'\n{3,}', '\n\n', cleaned)
        cleaned = cleaned.strip()

        return cleaned

    @classmethod
    def validate_no_metadata(cls, text: str) -> bool:
        """
        Check if text contains forbidden metadata.

        Args:
            text: Text to validate

        Returns:
            True if clean, False if metadata found
        """
        for pattern in cls.METADATA_PATTERNS:
            if re.search(pattern, text, re.IGNORECASE):
                return False
        return True
```

**Step 4.2: Apply in /chat endpoint**

Edit `fastapi_app/app.py`:

```python
from formatters import ResponseFormatter  # Add at top

@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    # ... after agent execution ...

    # Extract response
    agent_response = result.final_output if hasattr(result, 'final_output') else result.output

    # NEW: Strip metadata
    clean_response = ResponseFormatter.strip_metadata(agent_response)

    # NEW: Validate (log warning if metadata found)
    if not ResponseFormatter.validate_no_metadata(clean_response):
        logger.warning("Metadata detected in response after stripping")

    return AgentResponse(
        response=clean_response,
        session_id=session_id,
        source_chunks=formatted_chunks  # Keep for audit, not displayed
    )
```

**Test:**
```bash
# Manually add "Source: chunk_123" to agent response in code, then:
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test"}'

# Expected: "Source: chunk_123" is stripped from response
```

---

### Phase 5: Frontend - Thinking Animation (45 min)

**Goal:** Show smooth dot-wave animation during processing.

**Step 5.1: Create animation component**

Edit `src/theme/Root.tsx` (add CSS and component):

```typescript
// Add at top with other styles (inline or in <style> tag)
const thinkingAnimationStyles = `
  @keyframes wave {
    0%, 100% { opacity: 0.3; }
    50% { opacity: 1; }
  }

  .thinking-container {
    display: flex;
    justify-content: flex-start;
    padding: 1rem;
  }

  .thinking-bubble {
    background: #f1f5f9;
    border-radius: 12px;
    padding: 1rem 1.5rem;
    display: flex;
    align-items: center;
    gap: 0.75rem;
  }

  .thinking-text {
    color: #64748b;
    font-size: 0.95rem;
  }

  .thinking-dots {
    display: flex;
    gap: 0.25rem;
  }

  .thinking-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background-color: #94a3b8;
    animation: wave 1s ease-in-out infinite;
  }

  .thinking-dot:nth-child(1) { animation-delay: 0ms; }
  .thinking-dot:nth-child(2) { animation-delay: 150ms; }
  .thinking-dot:nth-child(3) { animation-delay: 300ms; }
`;

// Add ThinkingAnimation component
const ThinkingAnimation = () => (
  <div className="thinking-container">
    <div className="thinking-bubble">
      <span className="thinking-text">Thinking</span>
      <div className="thinking-dots">
        <div className="thinking-dot"></div>
        <div className="thinking-dot"></div>
        <div className="thinking-dot"></div>
      </div>
    </div>
  </div>
);
```

**Step 5.2: Add thinking state management**

```typescript
// Add state variable
const [isThinking, setIsThinking] = useState(false);

// Update sendMessage function
const sendMessage = async (question: string, isFromSelectedText = false) => {
    // Add user message to UI
    setMessages((prev) => [
        ...prev,
        { role: 'user', content: question, timestamp: new Date() },
    ]);

    // NEW: Start thinking animation
    setIsThinking(true);

    try {
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

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();

        // NEW: Stop thinking animation
        setIsThinking(false);

        // Add assistant response
        setMessages((prev) => [
            ...prev,
            {
                role: 'assistant',
                content: data.response,
                timestamp: new Date(),
            },
        ]);

    } catch (error) {
        // NEW: Stop thinking on error
        setIsThinking(false);

        setMessages((prev) => [
            ...prev,
            {
                role: 'assistant',
                content: `❌ **Error**: ${error.message}\\n\\nPlease check your connection and try again.`,
                timestamp: new Date(),
            },
        ]);
    }
};
```

**Step 5.3: Render animation in message area**

```typescript
// In render, after messages.map(), before input form:
{messages.map((message, index) => (
    // ... existing message rendering ...
))}

{/* NEW: Show thinking animation */}
{isThinking && <ThinkingAnimation />}

{/* Existing input form */}
<form onSubmit={handleSubmit}>
  ...
</form>
```

**Test:**
1. Open browser DevTools (F12)
2. Go to Network tab, throttle to "Slow 3G"
3. Send a message
4. Verify: Animation appears immediately and disappears when response arrives

---

### Phase 6: Testing & Validation (30 min)

**Run integration tests:**

```bash
# Test greeting detection
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"message": "hello"}'

# Test technical query
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"message": "What is SLAM?"}'

# Test metadata suppression (manually check response)
# Verify no "Source:", "chunk_", "Based on" appears

# Test thinking animation (browser)
# 1. Open frontend
# 2. Send message
# 3. Verify animation appears and disappears
```

**Manual checklist:**
- [ ] Greetings return friendly response without RAG
- [ ] Technical queries return markdown-formatted responses
- [ ] Responses have "## Topic", "### Key Points", "### Why It Matters" structure
- [ ] No metadata (chunk IDs, sources) in responses
- [ ] Thinking animation appears on message send
- [ ] Thinking animation disappears when response arrives
- [ ] Frontend error handling works (disconnect backend and test)

---

## Troubleshooting

### Issue: Greeting not detected

**Symptom:** "hello" triggers RAG instead of greeting response

**Solution:**
1. Check greeting_detector.py is imported
2. Verify if condition comes before RAG call
3. Add logging: `print(f"Is greeting: {GreetingDetector.is_greeting(request.message)}")`

---

### Issue: Groq fallback not working

**Symptom:** Error instead of fallback when Gemini quota exceeded

**Solution:**
1. Verify `GROQ_API_KEY` in `.env`
2. Check llm_router.py error detection logic
3. Add logging in except block: `logger.error(f"Fallback failed: {e}")`

---

### Issue: Metadata still appearing

**Symptom:** "Source: chunk_123" visible in response

**Solution:**
1. Check formatters.py is imported and called
2. Verify agent system prompt includes "Do NOT include chunk IDs"
3. Add logging: `logger.info(f"Before stripping: {agent_response[:200]}")`

---

### Issue: Thinking animation not appearing

**Symptom:** No animation when message sent

**Solution:**
1. Verify `<style>` tag with keyframe animation is in Root.tsx
2. Check `setIsThinking(true)` is called before fetch
3. Inspect browser DevTools: Elements tab for `.thinking-container`

---

## Next Steps

After completing this quickstart:
1. Run `/sp.tasks` to generate detailed implementation tasks
2. Run `/sp.implement` to execute tasks systematically
3. Test all edge cases from spec.md
4. Create pull request for review

---

## Support

**Documentation:**
- Full spec: `specs/006-agent-ui-enhancements/spec.md`
- Research: `specs/006-agent-ui-enhancements/research.md`
- API contract: `specs/006-agent-ui-enhancements/contracts/api-contract.md`

**Help:**
- GitHub Issues: Report bugs or ask questions
- Backend logs: Check `fastapi_app/` console output
- Frontend console: Check browser DevTools

**Estimated Total Implementation Time:** 3-4 hours
