# Tasks: Agent Behavior & UI Enhancement

**Input**: Design documents from `/specs/006-agent-ui-enhancements/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-contract.md

**Tests**: Manual testing + pytest integration tests (as specified in plan.md)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. P1 stories (Greeting, Formatting, Metadata) can be implemented first for MVP, followed by P2 stories (LLM Fallback, Thinking Animation).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `fastapi_app/` (Python/FastAPI)
- **Frontend**: `src/theme/` (React/TypeScript)
- **Tests**: `tests/` (pytest)
- **Config**: `.env.example`, `fastapi_app/requirements.txt`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment configuration

- [x] T001 Add GROQ_API_KEY to .env.example with documentation comment
- [x] T002 [P] Verify fastapi_app/requirements.txt has all dependencies (no new ones needed)
- [x] T003 [P] Create tests/ directory structure (tests/test_greeting_detection.py, tests/test_llm_fallback.py, tests/test_metadata_suppression.py)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core modules that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create fastapi_app/greeting_detector.py module with GreetingDetector class and GREETING_PATTERNS constant
- [x] T005 [P] Create fastapi_app/llm_router.py module with LLMRouter class and QuotaExceededError exception
- [x] T006 [P] Create fastapi_app/formatters.py module with ResponseFormatter class and metadata suppression patterns
- [x] T007 Configure Groq client in fastapi_app/connection.py (groq_client, groq_model, groq_config)

**Checkpoint**: ✅ Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Greeting Interaction (Priority: P1) 🎯 MVP

**Goal**: Detect greetings before RAG execution and return friendly welcome messages without triggering vector store queries or exposing metadata.

**Independent Test**: Send "hello", "hi", "salam" to /chat endpoint and verify response contains welcome message, source_chunks is empty, and no RAG-related logging occurs.

### Implementation for User Story 1

- [x] T008 [US1] Implement GreetingDetector.is_greeting() method in fastapi_app/greeting_detector.py with case-insensitive pattern matching
- [x] T009 [US1] Implement GreetingDetector.generate_greeting_response() method in fastapi_app/greeting_detector.py with structured markdown template
- [x] T010 [US1] Add greeting detection check in fastapi_app/app.py /chat endpoint BEFORE RAG retrieval (line ~350)
- [x] T011 [US1] Return AgentResponse with greeting text and empty source_chunks when greeting detected in fastapi_app/app.py
- [x] T012 [US1] Add logging statement "Greeting detected, skipping RAG" in fastapi_app/app.py

**Checkpoint**: At this point, greetings should bypass RAG and return friendly responses. Test independently: curl -X POST http://localhost:8000/chat -d '{"message": "hello"}'

---

## Phase 4: User Story 2 - Professional Response Formatting (Priority: P1) 🎯 MVP

**Goal**: Enforce structured markdown output with clear headings, bullet points, and consistent formatting across all non-greeting responses.

**Independent Test**: Ask "What is SLAM?" and verify response has "## Topic", "### Key Points", "### Why It Matters" sections with bullet-pointed lists.

### Implementation for User Story 2

- [x] T013 [US2] Update agent system prompt in fastapi_app/agent.py with RESPONSE STRUCTURE section (markdown template with ##, ###, bullet points)
- [x] T014 [US2] Add FORMATTING RULES to agent system prompt in fastapi_app/agent.py (paragraph length, tone, professional requirements)
- [x] T015 [US2] Add CONTENT RULES to agent system prompt in fastapi_app/agent.py (context-only, no hallucination, no metadata)
- [x] T016 [US2] Implement ResponseFormatter.validate_no_metadata() method in fastapi_app/formatters.py with regex checks
- [x] T017 [US2] Add logging warning when formatting validation fails in fastapi_app/app.py /chat endpoint

**Checkpoint**: At this point, all technical queries should return well-formatted markdown. Test independently: curl -X POST http://localhost:8000/chat -d '{"message": "What is inverse kinematics?"}'

---

## Phase 5: User Story 5 - Metadata Suppression (Priority: P1) 🎯 MVP

**Goal**: Strip all RAG metadata (chunk IDs, source references, internal identifiers) from user-facing responses while keeping source_chunks for internal audit.

**Independent Test**: Ask any technical question and grep response for "chunk_", "Source:", "Based on" - none should appear.

### Implementation for User Story 5

- [x] T018 [US5] Implement ResponseFormatter.strip_metadata() method in fastapi_app/formatters.py with regex patterns for forbidden metadata
- [x] T019 [US5] Add metadata stripping call in fastapi_app/app.py /chat endpoint after agent response extraction
- [x] T020 [US5] Add "Do NOT include chunk IDs, source references, or metadata" to PROHIBITED section in fastapi_app/agent.py system prompt
- [x] T021 [US5] Remove citations field from AgentResponse model in fastapi_app/app.py (if present - confirm first)
- [x] T022 [US5] Update frontend src/theme/Root.tsx to NOT display source_chunks array (remove any source rendering code)

**Checkpoint**: At this point, responses should be metadata-free. Test independently: curl -X POST http://localhost:8000/chat -d '{"message": "Explain SLAM"}' | grep -i "chunk"

---

## Phase 6: User Story 3 - Seamless LLM Fallback (Priority: P2)

**Goal**: Silent fallback from Gemini to Groq on quota errors without user-facing error messages or format changes.

**Independent Test**: Mock Gemini 429 error (temporarily add raise Exception("RESOURCE_EXHAUSTED") in code) and verify response still works with identical formatting.

### Implementation for User Story 3

- [x] T023 [US3] Implement LLMRouter.__init__() method in fastapi_app/llm_router.py with primary and fallback config parameters
- [x] T024 [US3] Implement LLMRouter.run_with_fallback() method in fastapi_app/llm_router.py with try-catch for quota errors (429, RESOURCE_EXHAUSTED)
- [x] T025 [US3] Add quota error detection logic in fastapi_app/llm_router.py (check error message for "quota", "429", "rate limit")
- [x] T026 [US3] Add fallback logging (warning for Gemini failure, info for Groq success) in fastapi_app/llm_router.py
- [x] T027 [US3] Replace direct Runner.run() call with llm_router.run_with_fallback() in fastapi_app/app.py /chat endpoint
- [x] T028 [US3] Add fallback_count tracking to LLMRouter class in fastapi_app/llm_router.py for monitoring

**Checkpoint**: At this point, Gemini quota errors should trigger silent Groq fallback. Test with mock error, verify logs show "falling back to Groq" and response still works.

---

## Phase 7: User Story 4 - Visual Thinking Indicator (Priority: P2)

**Goal**: Display smooth three-dot wave animation while agent is processing, disappearing immediately when response arrives.

**Independent Test**: Send message in browser, verify animation appears within 100ms, disappears within 50ms of response arrival, and runs at 60fps without UI freezing.

### Implementation for User Story 4

- [x] T029 [P] [US4] Add CSS keyframe animation for wave effect in src/theme/Root.tsx (0%, 50%, 100% opacity transitions)
- [x] T030 [P] [US4] Define .thinking-container, .thinking-bubble, .thinking-dots, .thinking-dot styles in src/theme/Root.tsx
- [x] T031 [P] [US4] Create ThinkingAnimation component in src/theme/Root.tsx with three animated dots
- [x] T032 [US4] Add isThinking state variable (useState<boolean>(false)) in src/theme/Root.tsx
- [x] T033 [US4] Set isThinking=true in sendMessage function BEFORE fetch call in src/theme/Root.tsx
- [x] T034 [US4] Set isThinking=false in sendMessage function AFTER response.json() in src/theme/Root.tsx
- [x] T035 [US4] Add isThinking=false in catch block for error handling in src/theme/Root.tsx
- [x] T036 [US4] Render ThinkingAnimation component conditionally when isThinking===true in src/theme/Root.tsx message area

**Checkpoint**: At this point, thinking animation should appear/disappear correctly. Test in browser with DevTools Network throttling, verify smooth animation and immediate stop on response.

---

## Phase 8: Integration Testing

**Purpose**: End-to-end validation of all user stories working together

- [x] T037 [P] Create tests/test_greeting_detection.py with test_greeting_detection() function (assert "Welcome" in response, source_chunks empty)
- [x] T038 [P] Create tests/test_metadata_suppression.py with test_metadata_suppression() function (assert no "chunk_", "source:" in response text)
- [x] T039 [P] Create tests/test_response_formatting.py with test_response_formatting() function (assert "##" and "###" and "Key Points" in response)
- [x] T040 [P] Create tests/test_llm_fallback.py with test_llm_fallback() async function using mocker to simulate Gemini error and Groq success
- [x] T041 Run all pytest tests (pytest tests/) and verify 100% pass rate
- [x] T042 Manual testing checklist from quickstart.md (greetings, formatting, metadata, fallback, animation)

**Checkpoint**: All tests passing, all user stories validated independently

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T043 [P] Update .env.example with ENABLE_GREETING_DETECTION, ENABLE_LLM_FALLBACK, ENABLE_METADATA_SUPPRESSION feature flags
- [x] T044 [P] Add docstrings to all new functions in greeting_detector.py, llm_router.py, formatters.py
- [x] T045 Remove any temporary debug logs or mock error injections from code
- [x] T046 [P] Test edge cases from spec.md (mixed greeting+question, ambiguous greetings, double-failure, rapid-fire messages)
- [x] T047 Verify no breaking API changes (request/response schemas unchanged except citations removal)
- [x] T048 Run quickstart.md validation (all 6 phases, verify 3-4 hour estimate accurate)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-7)**: All depend on Foundational phase completion
  - P1 stories (US1, US2, US5) can proceed in parallel after Foundation → **MVP**
  - P2 stories (US3, US4) can proceed after P1 stories or in parallel if staffed
- **Integration Testing (Phase 8)**: Depends on all desired user stories being complete
- **Polish (Phase 9)**: Depends on testing completion

### User Story Dependencies

- **US1 (Greeting)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **US2 (Formatting)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **US5 (Metadata)**: Can start after Foundational (Phase 2) - Works with US2 (both touch agent prompt)
- **US3 (LLM Fallback)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **US4 (Thinking Animation)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Tasks within a story generally sequential (except where marked [P])
- Agent prompt updates (US2, US5) should coordinate to avoid merge conflicts
- Frontend animation (US4) independent of all backend changes

### Parallel Opportunities

- **Phase 1 (Setup)**: All 3 tasks can run in parallel [P]
- **Phase 2 (Foundational)**: T005, T006, T007 can run in parallel [P] (T004 first)
- **Phase 3-7 (User Stories)**: All P1 stories (US1, US2, US5) can run in parallel after Foundation
- **Phase 8 (Integration)**: Test file creation (T037-T040) can run in parallel [P]
- **Phase 9 (Polish)**: T043, T044, T046 can run in parallel [P]

---

## Parallel Example: MVP (P1 Stories)

```bash
# After Phase 2 (Foundational) completes, launch all P1 stories in parallel:

# Developer A: User Story 1 (Greeting Interaction)
Task: "Implement GreetingDetector.is_greeting() method in fastapi_app/greeting_detector.py"
Task: "Implement GreetingDetector.generate_greeting_response() method in fastapi_app/greeting_detector.py"
Task: "Add greeting detection check in fastapi_app/app.py /chat endpoint"

# Developer B: User Story 2 (Professional Response Formatting)
Task: "Update agent system prompt in fastapi_app/agent.py with RESPONSE STRUCTURE"
Task: "Add FORMATTING RULES to agent system prompt in fastapi_app/agent.py"
Task: "Implement ResponseFormatter.validate_no_metadata() in fastapi_app/formatters.py"

# Developer C: User Story 5 (Metadata Suppression)
Task: "Implement ResponseFormatter.strip_metadata() in fastapi_app/formatters.py"
Task: "Add metadata stripping call in fastapi_app/app.py /chat endpoint"
Task: "Update frontend src/theme/Root.tsx to NOT display source_chunks"

# All three developers coordinate on fastapi_app/agent.py (US2 + US5 both touch system prompt)
```

---

## Parallel Example: P2 Stories (After MVP)

```bash
# After P1 stories complete, launch P2 stories in parallel:

# Developer D: User Story 3 (LLM Fallback)
Task: "Implement LLMRouter.run_with_fallback() in fastapi_app/llm_router.py"
Task: "Replace Runner.run() with llm_router.run_with_fallback() in fastapi_app/app.py"

# Developer E: User Story 4 (Thinking Animation)
Task: "Add CSS keyframe animation for wave effect in src/theme/Root.tsx"
Task: "Create ThinkingAnimation component in src/theme/Root.tsx"
Task: "Add isThinking state management in sendMessage function"

# These two stories are completely independent (backend vs frontend)
```

---

## Implementation Strategy

### MVP First (P1 Stories Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T007) - CRITICAL checkpoint
3. Complete Phase 3: User Story 1 - Greeting (T008-T012)
4. Complete Phase 4: User Story 2 - Formatting (T013-T017)
5. Complete Phase 5: User Story 5 - Metadata (T018-T022)
6. **STOP and VALIDATE**: Test all P1 stories independently
7. Run integration tests for P1 stories only
8. Deploy/demo MVP (greetings work, responses formatted, no metadata leakage)

**MVP Delivers:**
- ✅ Friendly greetings without RAG
- ✅ Professional markdown-formatted responses
- ✅ Zero metadata leakage
- ⏳ LLM fallback (not yet implemented)
- ⏳ Thinking animation (not yet implemented)

### Incremental Delivery (Add P2 Stories)

1. Complete MVP (Phases 1-5)
2. Add Phase 6: User Story 3 - LLM Fallback (T023-T028)
3. Test US3 independently → Deploy/Demo (now has resilience)
4. Add Phase 7: User Story 4 - Thinking Animation (T029-T036)
5. Test US4 independently → Deploy/Demo (now has UX polish)
6. Complete Phase 8: Integration Testing (T037-T042)
7. Complete Phase 9: Polish (T043-T048)

**Full Feature Delivers:**
- ✅ All MVP features
- ✅ Silent LLM fallback (Gemini → Groq)
- ✅ Smooth thinking animation
- ✅ Comprehensive testing
- ✅ Production-ready polish

### Parallel Team Strategy

With multiple developers (after Foundation completes):

1. **Team completes Setup + Foundational together** (T001-T007)
2. **Once Foundational is done, split work:**
   - Developer A: US1 (Greeting) - fastapi_app/greeting_detector.py, app.py
   - Developer B: US2 (Formatting) - fastapi_app/agent.py, formatters.py
   - Developer C: US5 (Metadata) - fastapi_app/formatters.py, app.py, src/theme/Root.tsx
   - Developer D: US3 (LLM Fallback) - fastapi_app/llm_router.py, app.py
   - Developer E: US4 (Thinking Animation) - src/theme/Root.tsx
3. **Coordinate on shared files:**
   - app.py: US1, US3, US5 all modify /chat endpoint (merge sequentially or coordinate timing)
   - agent.py: US2 and US5 both touch system prompt (merge carefully)
   - formatters.py: US2 and US5 both add methods (parallel OK, different functions)
   - Root.tsx: US4 and US5 both modify (coordinate - different sections)

---

## Task Summary

**Total Tasks**: 48 tasks across 9 phases

**Task Count by User Story**:
- US1 (Greeting - P1): 5 tasks (T008-T012)
- US2 (Formatting - P1): 5 tasks (T013-T017)
- US5 (Metadata - P1): 5 tasks (T018-T022)
- US3 (LLM Fallback - P2): 6 tasks (T023-T028)
- US4 (Thinking Animation - P2): 8 tasks (T029-T036)
- Setup: 3 tasks (T001-T003)
- Foundational: 4 tasks (T004-T007)
- Integration Testing: 6 tasks (T037-T042)
- Polish: 6 tasks (T043-T048)

**Parallel Opportunities Identified**:
- Phase 1: 2 tasks can run in parallel (T002, T003)
- Phase 2: 3 tasks can run in parallel (T005, T006, T007)
- After Foundation: All 5 user stories can start in parallel (24 implementation tasks)
- Phase 8: 4 test file creations can run in parallel (T037-T040)
- Phase 9: 3 polish tasks can run in parallel (T043, T044, T046)

**Independent Test Criteria for Each Story**:
- US1: curl test for greeting detection + empty source_chunks
- US2: curl test for markdown structure (##, ###, Key Points)
- US5: grep test for absence of metadata patterns
- US3: mock Gemini error test for silent fallback
- US4: browser test for animation timing and smoothness

**Suggested MVP Scope**: Phases 1-5 (US1, US2, US5) = 22 tasks
**Estimated MVP Time**: 2-3 hours (per quickstart.md estimates)
**Full Feature Time**: 4-5 hours (all 48 tasks)

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label maps task to specific user story for traceability and independent testing
- Each user story should be independently completable and testable before moving to next priority
- Commit after each task or logical group for easy rollback
- Stop at checkpoints to validate story independently
- Coordinate on shared files (app.py, agent.py) when multiple stories touch same file
- MVP delivers core value (P1 stories), P2 stories add resilience and UX polish

**Format Validation**: ✅ All tasks follow checklist format (checkbox, ID, optional [P], optional [Story], description with file path)
