# Data Model: Agent Behavior & UI Enhancement

**Feature**: 006-agent-ui-enhancements
**Date**: 2025-12-20
**Phase**: Phase 1 - Design

## Overview

This feature introduces new behavioral logic and UI state management without requiring new database tables or persistent storage. All entities are runtime constructs or configuration constants.

---

## Entities

### 1. GreetingPattern

**Type:** Configuration Constant (Set[str])
**Purpose:** Defines recognized greeting patterns for intent detection
**Lifecycle:** Static, loaded at application startup
**Persistence:** None (hardcoded)

**Structure:**
```python
class GreetingPattern:
    """Immutable set of greeting patterns."""

    PATTERNS: Set[str] = {
        "hi",
        "hello",
        "hey",
        "salam",
        "assalam o alaikum"
    }

    @classmethod
    def is_greeting(cls, message: str) -> bool:
        """
        Check if message matches a greeting pattern.

        Args:
            message: User message to check

        Returns:
            True if message is a greeting, False otherwise
        """
        cleaned = message.lower().strip()
        return cleaned in cls.PATTERNS
```

**Attributes:**
| Attribute | Type | Constraints | Description |
|-----------|------|-------------|-------------|
| PATTERNS | Set[str] | Immutable | Predefined greeting strings |

**Validation Rules:**
- All patterns stored in lowercase
- Exact string matching (case-insensitive)
- No partial matching (e.g., "hello world" ≠ "hello")

**Relationships:** None

---

### 2. ThinkingState

**Type:** Frontend UI State (React)
**Purpose:** Tracks whether agent is processing a request
**Lifecycle:** Per-session, resets on page reload
**Persistence:** None (in-memory React state)

**Structure:**
```typescript
interface ThinkingState {
    isThinking: boolean;      // Is agent currently processing?
    startedAt: Date | null;   // When did thinking start?
}

// React hook usage:
const [thinkingState, setThinkingState] = useState<ThinkingState>({
    isThinking: false,
    startedAt: null
});
```

**Attributes:**
| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| isThinking | boolean | Yes | false | Whether agent is processing |
| startedAt | Date \| null | Yes | null | Timestamp when thinking started |

**State Transitions:**
```
Initial: { isThinking: false, startedAt: null }
  ↓ (User sends message)
Thinking: { isThinking: true, startedAt: new Date() }
  ↓ (First response token arrives OR complete response received)
Ready: { isThinking: false, startedAt: null }
```

**Validation Rules:**
- `isThinking` must be boolean (never undefined/null)
- `startedAt` must be valid Date or null
- If `isThinking === true`, `startedAt` must not be null

**Relationships:**
- Tied to message send/receive cycle
- Controls thinking animation component visibility

---

### 3. LLMProvider

**Type:** Runtime Configuration Object
**Purpose:** Encapsulates LLM connection details and fallback logic
**Lifecycle:** Created at application startup, persists for application lifetime
**Persistence:** Configuration from environment variables

**Structure:**
```python
from enum import Enum
from dataclasses import dataclass
from typing import Optional

class LLMProviderType(Enum):
    GEMINI = "gemini"
    GROQ = "groq"

@dataclass
class LLMProviderConfig:
    """Configuration for a single LLM provider."""

    provider_type: LLMProviderType
    model_name: str
    api_key: str
    base_url: str
    priority: int  # Lower = higher priority (0 = primary, 1 = fallback)

@dataclass
class LLMProvider:
    """Manages LLM routing and fallback logic."""

    primary: LLMProviderConfig
    fallback: Optional[LLMProviderConfig]
    current_provider: LLMProviderType  # Tracks which provider is active
    fallback_count: int = 0  # Number of times fallback was triggered

    def get_active_config(self) -> LLMProviderConfig:
        """Returns configuration for currently active provider."""
        if self.current_provider == LLMProviderType.GEMINI:
            return self.primary
        return self.fallback

    def trigger_fallback(self) -> None:
        """Switch to fallback provider."""
        self.current_provider = LLMProviderType.GROQ
        self.fallback_count += 1
```

**Attributes:**
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| provider_type | LLMProviderType | Yes | Gemini or Groq |
| model_name | str | Yes | Model identifier (e.g., "gemini-2.5-flash") |
| api_key | str | Yes | API authentication key |
| base_url | str | Yes | API endpoint base URL |
| priority | int | Yes | 0 = primary, 1 = fallback |
| current_provider | LLMProviderType | Yes | Active provider |
| fallback_count | int | Yes | Fallback trigger counter (for monitoring) |

**Validation Rules:**
- `api_key` must not be empty
- `base_url` must be valid URL format
- `priority` must be 0 or 1
- Primary provider always has priority 0

**Example Instantiation:**
```python
gemini_config = LLMProviderConfig(
    provider_type=LLMProviderType.GEMINI,
    model_name="gemini-2.5-flash",
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    priority=0
)

groq_config = LLMProviderConfig(
    provider_type=LLMProviderType.GROQ,
    model_name="llama-3.1-70b-versatile",
    api_key=os.getenv("GROQ_API_KEY"),
    base_url="https://api.groq.com/openai/v1",
    priority=1
)

llm_provider = LLMProvider(
    primary=gemini_config,
    fallback=groq_config,
    current_provider=LLMProviderType.GEMINI
)
```

**Relationships:**
- Used by agent execution logic in `fastapi_app/connection.py`
- Interacts with OpenAI Agents SDK model configuration

---

### 4. ResponseEvent

**Type:** Backend Event Signal
**Purpose:** Signals frontend that response generation has started
**Lifecycle:** Ephemeral (created per request, discarded after signal sent)
**Persistence:** None

**Structure:**
```python
from enum import Enum
from datetime import datetime

class ResponseEventType(Enum):
    STARTED = "response_started"
    COMPLETED = "response_completed"
    ERROR = "response_error"

@dataclass
class ResponseEvent:
    """Event emitted by backend to signal response state."""

    event_type: ResponseEventType
    session_id: str
    timestamp: datetime
    metadata: dict = field(default_factory=dict)
```

**Attributes:**
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| event_type | ResponseEventType | Yes | Type of event (started/completed/error) |
| session_id | str | Yes | Session identifier for tracking |
| timestamp | datetime | Yes | When event occurred (ISO format) |
| metadata | dict | No | Additional context (error messages, etc.) |

**Validation Rules:**
- `event_type` must be valid ResponseEventType enum value
- `session_id` must not be empty
- `timestamp` must be valid ISO 8601 datetime

**Event Emission Points:**
```python
# In /chat endpoint (app.py):
async def chat(request: AgentRequest):
    session_id = request.session_id or str(uuid.uuid4())

    # Emit STARTED event (future: via SSE/WebSocket)
    emit_response_event(ResponseEvent(
        event_type=ResponseEventType.STARTED,
        session_id=session_id,
        timestamp=datetime.utcnow()
    ))

    # Process agent request...
    result = await Runner.run(...)

    # Emit COMPLETED event
    emit_response_event(ResponseEvent(
        event_type=ResponseEventType.COMPLETED,
        session_id=session_id,
        timestamp=datetime.utcnow()
    ))
```

**Note:** Current implementation doesn't use true events (no SSE/WebSocket). Frontend detects response start when HTTP response body begins.

**Relationships:**
- Correlates with `ThinkingState` on frontend
- Tied to `session_id` from existing SessionManager

---

### 5. AgentMessage (Extended)

**Type:** Response Data Transfer Object (DTO)
**Purpose:** Carries formatted agent response with metadata suppression
**Lifecycle:** Created per request, sent to frontend, discarded
**Persistence:** Stored in SessionManager history (in-memory)

**Structure (Extends Existing):**
```python
from pydantic import BaseModel, Field, field_validator

class QueryResult(BaseModel):
    """Retrieved chunk from RAG (internal use only)."""
    id: str
    content: str
    similarity_score: float
    metadata: dict = {}

class AgentMessage(BaseModel):
    """Agent response with metadata suppression."""

    response: str = Field(
        ...,
        description="Formatted agent answer (markdown)",
        min_length=1
    )
    session_id: str = Field(
        ...,
        description="Session identifier"
    )

    # Internal fields (not exposed to frontend in display)
    source_chunks: List[QueryResult] = Field(
        default_factory=list,
        description="Retrieved chunks (for audit, not user display)"
    )

    # Removed: citations field (no longer exposed)
    # Old: citations: List[str] = []

    @field_validator('response')
    def validate_no_metadata(cls, v):
        """Ensure response doesn't contain RAG metadata."""
        forbidden_patterns = [
            r'source:\s*chunk_\w+',
            r'based on chunk',
            r'chunk id',
            r'similarity score',
            r'retrieved from'
        ]

        import re
        for pattern in forbidden_patterns:
            if re.search(pattern, v, re.IGNORECASE):
                raise ValueError(f"Response contains forbidden metadata pattern: {pattern}")

        return v
```

**Attributes:**
| Attribute | Type | Required | Exposed to Frontend | Description |
|-----------|------|----------|---------------------|-------------|
| response | str | Yes | Yes | Formatted markdown answer |
| session_id | str | Yes | Yes | Session tracking |
| source_chunks | List[QueryResult] | No | No (internal only) | Retrieved RAG chunks for audit |

**Changes from Existing:**
- **Removed:** `citations` field (previously: `List[str]`)
- **Added:** Pydantic validator to enforce metadata suppression
- **Modified:** `source_chunks` marked as internal (not displayed in frontend)

**Validation Rules:**
- `response` must not contain:
  - "Source: chunk_*"
  - "Based on chunk"
  - "Chunk ID"
  - "Similarity score"
  - "Retrieved from"
- `response` must not be empty
- `session_id` must be valid UUID format

**Formatting Requirements:**
```python
# Response must follow this structure:
## {Topic Title}

{Brief 2-3 sentence explanation}

### Key Points
- {Point 1}
- {Point 2}
- {Point 3}

### Why It Matters
- {Practical relevance}
- {Application context}
```

**Relationships:**
- Created in `/chat` endpoint after agent execution
- Stored in `SessionManager.sessions` history
- Sent to frontend as JSON response

---

## State Transitions

### Message Processing Flow

```
[User sends message]
    ↓
[Frontend: isThinking = true]
    ↓
[Backend: Check if greeting]
    ├─ Yes → [Generate greeting response (no RAG)]
    └─ No → [Execute RAG search → Run agent]
        ↓
    [LLM Provider: Try Gemini]
        ├─ Success → [Format response]
        └─ Quota Error → [Fallback to Groq] → [Format response]
            ↓
    [Backend: Strip RAG metadata]
        ↓
    [Backend: Return AgentMessage]
        ↓
[Frontend: Receive response]
    ↓
[Frontend: isThinking = false]
    ↓
[Display formatted message]
```

---

## Configuration Constants

### Feature Flags

```python
@dataclass
class FeatureFlags:
    """Runtime feature toggles."""

    enable_greeting_detection: bool = True
    enable_llm_fallback: bool = True
    enable_metadata_suppression: bool = True
    enable_response_formatting: bool = True

    @classmethod
    def from_env(cls):
        return cls(
            enable_greeting_detection=os.getenv("ENABLE_GREETING_DETECTION", "true").lower() == "true",
            enable_llm_fallback=os.getenv("ENABLE_LLM_FALLBACK", "true").lower() == "true",
            enable_metadata_suppression=os.getenv("ENABLE_METADATA_SUPPRESSION", "true").lower() == "true",
            enable_response_formatting=os.getenv("ENABLE_RESPONSE_FORMATTING", "true").lower() == "true",
        )
```

---

## Summary

This feature introduces **5 new entities**:
1. **GreetingPattern** - Static configuration for intent detection
2. **ThinkingState** - Frontend UI state for animation control
3. **LLMProvider** - Runtime configuration for LLM routing
4. **ResponseEvent** - Backend event signal (future enhancement)
5. **AgentMessage** - Extended DTO with metadata suppression

**No database changes required.** All entities are runtime constructs or configuration constants.

**Existing entities modified:**
- `AgentMessage` (removed `citations` field, added validation)

**Existing entities preserved:**
- `SessionManager` - No changes
- `QueryResult` - No changes
- `ConnectionManager` - No changes
