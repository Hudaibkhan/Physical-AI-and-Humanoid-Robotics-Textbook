---
id: 1
title: better-auth-personalization-plan
stage: plan
date: 2025-12-17
surface: agent
model: gemini-2.5-flash
feature: better-auth-personalization
branch: 005-better-auth-personalization
user: Muhammad Hudaib Arif
command: /sp.plan
labels: [auth, personalization, better-auth, neon-db, plan]
links:
  spec: specs/005-better-auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-better-auth-personalization/plan.md
  - specs/005-better-auth-personalization/research.md
  - specs/005-better-auth-personalization/data-model.md
  - specs/005-better-auth-personalization/quickstart.md
  - specs/005-better-auth-personalization/contracts/auth-api-contracts.md
tests:
  - specs/005-better-auth-personalization/checklists/requirements.md
---

## Prompt

Better Auth + Neon DB + Personalization Fix Plan

This plan defines the exact steps Claude Code must follow to fix authentication, database persistence, and content personalization without modifying chatbot or backend logic.

---

## Phase 1: Documentation & Context Validation

1. Call MCP to fetch Better Auth documentation:
context7 - resolve-library-id (MCP)(libraryName: "better-auth")

yaml
Copy code

2. Read and extract:
- Proper AuthProvider usage
- Correct signup / signin API
- How metadata/profile data is stored
- Neon DB adapter integration
- Common failure causes for registration

3. Validate current implementation against docs:
- Ensure `AuthProvider` wraps the app correctly
- Confirm `useAuth` is not called outside provider
- Identify why registration is failing silently

---

## Phase 2: Fix Signup Failure

1. Inspect signup flow:
- Check request payload
- Check required fields
- Check adapter configuration

2. Verify signup fields:
- email
- password
- software_background
- hardware_background
- learning_goal

3. Fix causes of "Registration failed":
- Missing fields
- Wrong schema
- Adapter mismatch
- Neon DB write failure

4. Add:
- Proper error messages
- Loading states
- Validation feedback

---

## Phase 3: Neon DB Persistence Fix

1. Verify Neon DB connection:
- Connection string
- Adapter initialization

2. Check database schema:
- Users table
- Profile fields

3. Fix data write issues:
- Ensure profile metadata is saved on signup
- Ensure profile data persists after login

4. Add verification:
- Fetch user profile after login
- Confirm data consistency

---

## Phase 4: Auth Context Crash Fix

1. Fix error:
useAuth must be used within an AuthProvider

yaml
Copy code

2. Ensure:
- Single AuthProvider at root level
- No duplicate providers
- No hooks used outside provider

3. Test:
- App load
- Page refresh
- Route navigation

---

## Phase 5: UI & UX Improvements

### Signup / Login Pages
- Improve layout and spacing
- Clear field labels
- Helpful error text
- Disabled submit during loading

### Auth State UI
- Show profile / personalize button only when logged in
- Hide for logged-out users

---

## Phase 6: Personalization Logic Fix

1. After login:
- Fetch user profile from Neon DB
- Store profile in client state

2. Create personalization view/page:
- Display stored profile data
- Explain personalization behavior

3. Chapter-level personalization:
- Fix personalization button behavior
- Ensure correct content switching
- Do NOT overwrite original content
- Ensure toggle works per chapter

---

## Phase 7: Behavioral Validation

1. Logged-out user:
- No personalization controls
- Normal content only

2. Logged-in user:
- Profile button visible
- Personalization works correctly

3. Ensure:
- No chatbot changes
- No fastapi_app changes
- No console errors

---

## Phase 8: Final Testing & Verification

1. Test flows:
- Signup
- Login
- Logout
- Page reload
- Personalization toggle

2. Verify:
- Neon DB contains user data
- UI reflects auth state
- Content personalization behaves correctly

3. Provide final report:
- Issues fixed
- Files changed
- DB verification summary

---

## Completion Rule

This plan is complete only when:
- Signup succeeds
- Login succeeds
- Neon DB persists user profile
- Personalization w

## Response snapshot

# Implementation Plan: Better Auth + Neon DB + Personalization Fix

**Branch**: `005-better-auth-personalization` | **Date**: 2025-12-17 | **Spec**: [specs/005-better-auth-personalization/spec.md](../specs/005-better-auth-personalization/spec.md)
**Input**: Feature specification from `/specs/005-better-auth-personalization/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement Better Auth with Neon DB integration for user signup/login, collect user profile data (software_background, hardware_background, learning_goal) during registration, and enable content personalization for logged-in users. Fix current auth context crash and improve UI/UX for authentication flows.

## Technical Context

**Language/Version**: TypeScript/JavaScript with Node.js v22+
**Primary Dependencies**: Better Auth SDK, Docusaurus, React, Neon Postgres
**Storage**: Neon Postgres database via Better Auth adapter
**Testing**: N/A (manual testing for this feature)
**Target Platform**: Web application (Docusaurus frontend with Express.js backend)
**Project Type**: Web application - frontend (Docusaurus) with backend API server
**Performance Goals**: <2s auth operations, <500ms personalization toggle
**Constraints**: Must not modify chatbot or FastAPI backend, maintain existing UI structure
**Scale/Scope**: Individual textbook platform, single database

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] Agent Architecture: No changes to OpenAI Agents SDK components required
- [x] MCP Integration: No MCP tool changes needed for auth functionality
- [x] Production-Ready Implementation: All auth code will be production-ready
- [x] Agent Response Quality: No changes to agent response handling
- [x] System Consistency: Auth flows will maintain consistency with existing UI
- [x] Vector Database: No changes to Qdrant integration
- [x] Chat Frontend: No changes to ChatKit widget
- [x] Backend Architecture: Changes limited to auth-related endpoints

## Project Structure

### Documentation (this feature)

```text
specs/005-better-auth-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── auth/
│   ├── client.ts              # Better Auth client configuration
│   ├── auth.config.ts         # Better Auth server configuration
│   └── context/
│       └── AuthContext.tsx    # Authentication context provider
├── pages/
│   └── auth.js                # Authentication page (login/register)
├── theme/
│   └── Root.tsx               # Docusaurus root component with AuthProvider
├── personalization/
│   └── components/
│       └── PersonalizeChapterButton.tsx  # Personalization UI component
├── database/
│   └── db.utils.ts            # Database utilities
└── components/
    └── NavbarAuth.js          # Auth-aware navigation component

api-server.js                   # Express.js server with Better Auth integration
.env                           # Environment variables including Neon DB URL
package.json                   # Dependencies including better-auth
```

**Structure Decision**: Web application with Docusaurus frontend and Express.js backend API server. Authentication is handled through Better Auth with Neon DB adapter, and personalization is implemented as React components integrated with Docusaurus theme.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Outcome

- ✅ Impact: Created comprehensive implementation plan for Better Auth + Neon DB + Personalization
- 🧪 Tests: Validated plan against project constitution with all checks passing
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and API contracts
- 🔁 Next prompts: Ready for task generation with `/sp.tasks`
- 🧠 Reflection: Plan addresses all requirements from user input with clear technical approach

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All constitution checks passed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks with `/sp.tasks`