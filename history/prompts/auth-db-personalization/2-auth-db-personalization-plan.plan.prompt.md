---
id: 2
title: auth-db-personalization-plan
stage: plan
date: 2025-12-16
surface: agent
model: gemini-2.5-flash
feature: auth-db-personalization
branch: 004-auth-db-personalization
user: Claude Code
command: /sp.plan
labels: ["authentication", "personalization", "database", "planning"]
links:
  spec: specs/004-auth-db-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-auth-db-personalization/plan.md
 - specs/004-auth-db-personalization/research.md
 - specs/004-auth-db-personalization/data-model.md
 - specs/004-auth-db-personalization/contracts/personalization-api.yaml
 - specs/004-auth-db-personalization/quickstart.md
 - .specify/memory/agent-context-auth-personalization.md
tests:
 - plan completeness validation
 - data model accuracy
 - API contract verification
---

## Prompt

```
# Book Platform – Auth, DB & Personalization Implementation Plan (With Explicit Signup Questions)

This plan defines step-by-step tasks Claude Code + SpecKit Plus should follow to implement Better Auth integration, content personalization, and database-backed state, without touching existing chatbot code or backend.

---

## 1️⃣ Prerequisites (Manual, Outside Prompt)

- Ensure Better Auth project exists and API keys are available
- Ensure Neon Postgres DB exists and DATABASE_URL is ready
- Environment variables set:
  - BETTER_AUTH_PUBLIC_KEY
  - BETTER_AUTH_SECRET_KEY
  - DATABASE_URL

---

## 2️⃣ Step-by-Step Plan

### Step 1 – MCP Context7

- Call MCP context7 to fetch OpenAI Agents SDK documentation
- Ensure that no agent creation or tool modification is done
- Use documentation only for reference if needed

---

### Step 2 – Database Schema Setup

- Generate SQL schema for:
  - `user_chapter_state`
  - `personalized_content_cache` (optional)
- Include queries for:
  - Insert / update personalization state
  - Fetch personalization state per user per chapter
- Ensure database connection uses `DATABASE_URL` env variable

---

### Step 3 – Authentication Integration (Explicit Fields)

- Implement Better Auth frontend integration
- **Signup form must include:**
  1. Email (required)
  2. Password (required)
  3. Skill level (metadata)
  4. Hardware background (metadata)
  5. Learning goal (metadata)
- Signin form: Email + Password
- Store metadata in auth provider
- Backend only verifies tokens and reads metadata
- Do NOT touch FastAPI chatbot endpoints

---

### Step 4 – Book UI Personalization Flow

- Add "Personalize this chapter" button at chapter start
- On click:
  - Send user_id, chapter_id, and user metadata to backend endpoint (e.g., `/personalize`)
  - Backend queries database for previous personalization state
  - Backend triggers agent (existing book_rag_agent) **with metadata** to generate personalized content
  - Return personalized content to frontend
- Render personalized content while keeping citations intact

---

### Step 5 – Backend Endpoints (Extensions Only)

- Create new endpoint `/personalize` (or reuse existing safe route)
  - Accept user_id, chapter_id, and optional metadata
  - Save / fetch personalization state
  - Call agent with user context for personalized answer
- Do NOT modify `/chat` endpoint

---

### Step 6 – Frontend Integration

- Ensure login state is maintained
- Selected chapter personalization requests trigger backend endpoint
- Display returned personalized content in the chapter
- Keep existing chatbot widget untouched

---

### Step 7 – Testing & Validation

- Test user signup/signin flows including:
  - Email + Password
  - Skill level, hardware, learning goal metadata
- Test personalization per chapter
- Test database CRUD for personalization state
- Ensure chatbot still answers exactly as before
- Ensure no breaking changes in fastapi_app folder

---

### Step 8 – Cleanup & Output

- Generate updated project structure:
  - Auth frontend integration files with explicit signup questions
  - Database schema & queries
  - Personalization logic & components
- Ensure no modifications to:
  - fastapi_app/
  - Chatbot agent
  - Existing RAG retrieval
```

## Response snapshot

Successfully created a comprehensive implementation plan for the Book Platform authentication and personalization feature. The plan includes:

1. **Technical Context**: Defined technologies (Better Auth, Neon Postgres, existing book_rag_agent) and resolved all unknowns through research
2. **Constitution Check**: Verified compliance with project constitution and defined implementation gates
3. **Phase 0 Research**: Completed research on MCP context7, project structure, and Better Auth integration
4. **Phase 1 Design**: Created data model documentation, API contracts (OpenAPI spec), and quickstart guide
5. **Agent Context**: Updated agent context with new technologies for the feature

All artifacts created:
- Implementation plan (plan.md)
- Research findings (research.md)
- Data model documentation (data-model.md)
- API contracts (personalization-api.yaml)
- Quickstart guide (quickstart.md)
- Agent context updates (.specify/memory/agent-context-auth-personalization.md)

The plan follows the required sequence and maintains strict separation from existing chatbot functionality as specified.

## Outcome

- ✅ Impact: Complete implementation plan created with all required artifacts
- 🧪 Tests: Plan validated against constitution requirements
- 📁 Files: All required documentation and contracts generated
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Plan successfully balances new feature requirements with existing system constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All plan components validated
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks from plan