---
id: 1
title: auth-db-personalization-spec
stage: spec
date: 2025-12-16
surface: agent
model: gemini-2.5-flash
feature: auth-db-personalization
branch: 004-auth-db-personalization
user: Claude Code
command: /sp.specify
labels: ["authentication", "personalization", "database"]
links:
  spec: specs/004-auth-db-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-auth-db-personalization/spec.md
 - specs/004-auth-db-personalization/checklists/requirements.md
tests:
 - spec validation checklist
---

## Prompt

```
# Book Platform – Auth, DB & Personalization Specification (Non-Invasive)

This specification defines how Claude Code + SpecKit Plus must extend the existing book platform by adding authentication, database-backed personalization, and chapter-level content customization, WITHOUT modifying the existing chatbot or FastAPI backend implementation.

---

## 🎯 Goal

- Add Signup / Signin using Better Auth
- Enable content personalization for logged-in users
- Store personalization state in a database
- Integrate personalization with the book site UI
- Keep existing chatbot, agent, and backend code unchanged

---

## 1. Strict Non-Modification Rules

Claude Code MUST NOT:

- Modify any files inside:
  - fastapi_app/
  - chatbot-related folders
- Change:
  - OpenAI Agents SDK implementation
  - Qdrant logic
  - Agent tools or prompts
  - FastAPI chatbot endpoints
- Refactor or rewrite existing chatbot code

Chatbot behavior must remain exactly as-is.

---

## 2. Authentication

- Use K documentation
- MCP must NOT be used for personalization, auth, or database logic
- No new agent or tool creation is allowed under this spec

---

## 7. Stability & Safety Rules

- Existing chatbot must continue to work without changes
- fastapi_apBetter Auth SDK for Signup and Signin
- Assume auth credentials are provided via environment variables
- Collect user background data at signup:
  - skill_level
  - hardware_background
  - learning_goal
- Store background data as auth user metadata
- Authentication logic must live outside the chatbot backend

---

## 3. Database Integration

- Use Neon Postgres via DATABASE_URL
- Database is used ONLY for personalization state
- Do NOT store:
  - Book content
  - Embeddings
  - Chat history
  - Auth credentials

### Required Tables

#### user_chapter_state
- user_id (TEXT)
- chapter_id (TEXT)
- personalization_level (TEXT)
- language (TEXT)
- updated_at (TIMESTAMP)

#### personalized_content_cache (optional)
- user_id (TEXT)
- chapter_id (TEXT)
- content (TEXT)

---

## 4. Book Content Personalization

- Add a "Personalize this chapter" button at the start of each chapter
- Personalization is applied ONLY to book page content
- Personalized content must:
  - Remain grounded in existing chapter text
  - Rep folder must remain untouched
- .env files must not be deleted or modified
- No breaking changes allowed

---

## 8. Output Expectations

Claude Code must generate:
- Better Auth frontend integration
- Database schema & queries for personalization
- Book UI personalization logic
- Zero changes to existing chatbot code
```

## Response snapshot

Created a comprehensive feature specification for the book platform authentication and personalization feature. The specification includes:

1. User stories for registration with background information (P1), chapter personalization (P1), and state persistence (P2)
2. Functional requirements covering auth integration, background data collection, personalization features, and database storage
3. Key entities (User, Chapter, UserChapterState, PersonalizedContentCache)
4. Success criteria with measurable outcomes for user engagement, performance, and system reliability
5. Edge cases for unauthenticated users, database failures, and preference updates

The specification was created with the following details:
- Branch name: 004-auth-db-personalization
- Spec file: specs/004-auth-db-personalization/spec.md
- Quality checklist: specs/004-auth-db-personalization/checklists/requirements.md

The specification adheres to all non-modification rules, ensuring existing chatbot and FastAPI backend implementations remain unchanged.

## Outcome

- ✅ Impact: Created complete feature specification for authentication and personalization system
- 🧪 Tests: Validated with specification quality checklist
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification successfully captures all requirements while maintaining non-invasive approach to existing systems

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All checklist items validated
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase