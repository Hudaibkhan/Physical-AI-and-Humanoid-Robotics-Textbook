# Feature Specification: Agent Behavior & UI Enhancement

**Feature Branch**: `006-agent-ui-enhancements`
**Created**: 2025-12-20
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics Agent - Behavior, Formatting, LLM Routing, and Thinking-State UI Animation Update"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Greeting Interaction (Priority: P1)

A student opens the chatbot and types "hello" or "salam" to initiate a conversation. The agent responds naturally with a welcoming message that sets context about its capabilities, without triggering unnecessary RAG retrieval or exposing internal metadata.

**Why this priority**: First impressions matter. Greeting handling is the entry point for every user interaction and sets expectations for the entire experience. A friendly, contextual greeting without technical glitches establishes trust.

**Independent Test**: Can be fully tested by sending various greeting messages (hi, hello, hey, salam, assalam o alaikum) and verifying the agent responds appropriately without triggering RAG or showing internal metadata.

**Acceptance Scenarios**:

1. **Given** a user opens the chat interface, **When** they send "hello", **Then** the agent responds with a friendly greeting mentioning Physical AI, ROS 2, or Humanoid Robotics capabilities without RAG metadata
2. **Given** a user sends "salam", **When** the message is processed, **Then** the agent detects it as a greeting and responds appropriately without querying the vector store
3. **Given** a user sends "hey there!", **When** the agent processes it, **Then** the response includes an invitation to ask technical questions and does not reference textbook chunks or sources

---

### User Story 2 - Professional Response Formatting (Priority: P1)

A student asks a technical question like "What is inverse kinematics?" and receives a well-structured, professional response with clear headings, bullet points, and concise explanations that are easy to read and understand.

**Why this priority**: Information delivery quality directly impacts learning effectiveness. Well-formatted responses improve comprehension and reduce cognitive load, making complex robotics concepts more accessible.

**Independent Test**: Can be tested by asking any technical question and verifying the response follows the structured markdown template with headings, key points, and "Why It Matters" sections.

**Acceptance Scenarios**:

1. **Given** a user asks a technical question, **When** the agent generates a response, **Then** the output includes markdown headings (## Topic, ### Key Points, ### Why It Matters)
2. **Given** the agent retrieves multiple pieces of information, **When** formatting the response, **Then** content is organized with bullet points and short paragraphs (max 3-4 sentences each)
3. **Given** a complex topic requires explanation, **When** the response is generated, **Then** the structure follows: brief explanation, key points list, and practical relevance section

---

### User Story 3 - Seamless LLM Fallback (Priority: P2)

During peak usage, the Gemini API quota is exceeded. The system automatically switches to Groq without user awareness or error messages, maintaining service continuity and response quality.

**Why this priority**: System reliability is critical for user trust. Silent fallback ensures uninterrupted service without exposing infrastructure details or causing user anxiety about service availability.

**Independent Test**: Can be tested by simulating Gemini quota exhaustion (mock API response) and verifying responses continue without error messages and maintain identical format.

**Acceptance Scenarios**:

1. **Given** Gemini API returns a quota exceeded error, **When** the agent processes a request, **Then** it retries with Groq automatically without notifying the user
2. **Given** Groq is being used as fallback, **When** a response is generated, **Then** the output format remains identical to Gemini responses
3. **Given** a fallback occurs, **When** the user receives the response, **Then** no system messages about model switching appear

---

### User Story 4 - Visual Thinking Indicator (Priority: P2)

A user submits a question and immediately sees a smooth, animated dot-wave indicator showing the agent is processing. The animation disappears the moment the first response tokens arrive, creating a responsive, modern UX.

**Why this priority**: Visual feedback reduces perceived wait time and anxiety. Users need confirmation their request is being processed, especially for complex queries that take 3-5 seconds to process.

**Independent Test**: Can be tested by sending any query and observing the UI displays the dot-wave animation until the first response token arrives, then immediately transitions to showing the response.

**Acceptance Scenarios**:

1. **Given** a user submits a message, **When** the request is sent to the backend, **Then** a three-dot wave animation appears in the chat interface
2. **Given** the thinking animation is displaying, **When** the first response token arrives from the agent, **Then** the animation disappears immediately and the response text begins rendering
3. **Given** a slow network connection, **When** response streaming is delayed, **Then** the animation continues smoothly without freezing or jumping

---

### User Story 5 - Metadata Suppression (Priority: P1)

A user asks "Explain ROS 2 navigation stack" and receives a clean, authoritative response that appears fully authored by the agent, without any references to chunk IDs, source metadata, or internal RAG system details.

**Why this priority**: Professional presentation is essential for credibility. Exposing internal system details undermines the agent's authority and distracts from content quality, making responses feel machine-generated rather than expert-authored.

**Independent Test**: Can be tested by asking questions that trigger RAG retrieval and verifying the response contains no chunk IDs, source metadata, document references, or RAG system identifiers.

**Acceptance Scenarios**:

1. **Given** the agent retrieves information from the vector store, **When** generating a response, **Then** no chunk IDs or internal identifiers appear in the output
2. **Given** multiple documents contribute to an answer, **When** the response is formatted, **Then** no "Source:" or "Based on chunk X" references are included
3. **Given** RAG metadata exists in retrieved content, **When** the agent processes it, **Then** all metadata is stripped before generating the user-facing response

---

### Edge Cases

- What happens when a user sends a greeting followed immediately by a question in one message ("Hi, what is SLAM?")?
- How does the system handle ambiguous greetings like "sup" or "yo"?
- What occurs if Groq also returns an error during fallback?
- How does the UI behave if the WebSocket connection drops during streaming?
- What happens if the backend returns an error instead of a response start signal?
- How does the thinking animation handle rapid-fire consecutive messages?
- What occurs if RAG metadata appears in the middle of technical content (embedded in text)?

## Requirements *(mandatory)*

### Functional Requirements

#### Agent Behavior

- **FR-001**: System MUST detect greeting intent from a predefined list (hi, hello, hey, salam, assalam o alaikum) before executing RAG retrieval
- **FR-002**: System MUST respond to greetings with a friendly message mentioning Physical AI, ROS 2, or Humanoid Robotics without consulting the vector store
- **FR-003**: System MUST format all non-greeting responses using markdown with clear headings (## for topic, ### for subsections)
- **FR-004**: System MUST structure responses with: brief explanation, key points section (bulleted), and "Why It Matters" section
- **FR-005**: System MUST limit paragraph length to 3-4 sentences maximum for readability
- **FR-006**: System MUST strip all RAG metadata (chunk IDs, source references, document identifiers) from responses before presenting to users
- **FR-007**: System MUST ensure final responses appear fully authored without internal system references
- **FR-008**: Agent persona MUST convey: robotics instructor tone, friendly and calm demeanor, confidence and precision, clean professionalism

#### LLM Routing & Fallback

- **FR-009**: System MUST attempt Gemini (free tier) as the primary LLM for all requests
- **FR-010**: System MUST detect Gemini quota exceeded errors and automatically retry with Groq
- **FR-011**: System MUST maintain identical response formatting between Gemini and Groq outputs
- **FR-012**: System MUST never display user-facing messages about model switching, quota limits, or LLM selection
- **FR-013**: System MUST log fallback events server-side for monitoring (without exposing to users)

#### Frontend UI - Thinking State

- **FR-014**: UI MUST display a three-dot wave animation when a request is sent and no response has been received
- **FR-015**: Animation MUST loop smoothly every ~1 second with wave/pulse/fade effect
- **FR-016**: Animation MUST use neutral colors (gray or theme-based) without bright/distracting hues
- **FR-017**: UI MUST remove the thinking animation immediately upon receiving the first response token or complete message
- **FR-018**: UI MUST NOT display "Loading...", spinners, or technical status messages during thinking state
- **FR-019**: UI MUST NOT freeze or block user interaction during thinking state (e.g., allow scrolling previous messages)

#### Backend-Frontend Coordination

- **FR-020**: Backend MUST emit a clear "response started" signal via streaming token or event
- **FR-021**: Frontend MUST listen for response start signal and update UI state accordingly
- **FR-022**: System MUST handle streaming responses by removing animation on first token
- **FR-023**: System MUST handle non-streaming responses by removing animation when complete message arrives

#### Integration Constraints

- **FR-024**: System MUST preserve all existing FastAPI endpoints without breaking changes
- **FR-025**: System MUST preserve the existing RAG pipeline architecture (vector store, retriever)
- **FR-026**: System MUST NOT introduce paid dependencies or libraries
- **FR-027**: System MUST NOT expose backend errors (stack traces, API keys, internal paths) to the UI
- **FR-028**: System MUST maintain compatibility with OpenAI Agents SDK currently in use

### Key Entities *(include if feature involves data)*

- **Greeting Pattern**: A defined set of string patterns (hi, hello, hey, salam, assalam o alaikum) used to detect greeting intent before RAG execution
- **Agent Message**: The formatted response object containing markdown content, agent persona tone, and metadata-stripped text
- **Thinking State**: A boolean UI state indicating whether a request is in-flight and no response has been received
- **LLM Provider**: The selected model service (Gemini or Groq) with fallback logic and error handling
- **Response Event**: A signal from the backend indicating response generation has started, used to trigger UI state transition

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of greeting messages (hi, hello, hey, salam, assalam o alaikum) receive context-appropriate responses without triggering RAG retrieval
- **SC-002**: 100% of non-greeting responses follow the structured markdown template (heading, key points, why it matters)
- **SC-003**: 0% of responses contain visible RAG metadata (chunk IDs, source references, internal identifiers)
- **SC-004**: Gemini-to-Groq fallback completes within 2 seconds with no user-visible error messages
- **SC-005**: Thinking animation appears within 100ms of message send and disappears within 50ms of first response token
- **SC-006**: Users perceive responses as professionally authored rather than machine-generated (subjective but testable via user feedback)
- **SC-007**: System maintains response time of 3-5 seconds for typical queries with visible progress indication throughout
- **SC-008**: 0% of responses expose backend error details (API errors, stack traces, infrastructure information)

### Assumptions

- Gemini free tier quota is sufficient for most usage but will occasionally be exceeded during peak times
- Groq free tier provides comparable response quality and formatting capability to Gemini
- Existing FastAPI backend supports streaming responses or event-based signaling
- Current frontend uses HTTP or WebSocket transport capable of receiving streaming tokens
- The OpenAI Agents SDK allows custom LLM provider integration and fallback logic
- Vector store retrieval can be conditionally bypassed for greeting detection
- Frontend framework supports CSS animations or JavaScript-based animation rendering
- Users have modern browsers capable of rendering smooth CSS animations (no IE11 support required)
