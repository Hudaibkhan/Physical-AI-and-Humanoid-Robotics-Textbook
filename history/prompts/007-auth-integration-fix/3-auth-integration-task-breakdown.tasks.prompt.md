---
id: 3
title: Auth Integration Task Breakdown
stage: tasks
date: 2025-12-20
surface: agent
model: claude-haiku-4-5
feature: 007-auth-integration-fix
branch: 007-auth-integration-fix
user: system
command: /sp.tasks
labels: ["tasks", "implementation", "breakdown", "user-stories"]
links:
  spec: specs/007-auth-integration-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-auth-integration-fix/tasks.md
tests:
  - Manual E2E testing specified (no automated tests required)
---

## Prompt

Generate implementation tasks for Auth Integration & Book Crash Fix feature based on:
- User stories from spec.md (5 stories: P1, P2, P2, P3, P3)
- Technical architecture from plan.md
- Database schema and decisions from research.md

## Response snapshot

Generated comprehensive task breakdown for Auth Integration & Book Crash Fix (feature 007-auth-integration-fix).

**Tasks Document**: `specs/007-auth-integration-fix/tasks.md`

**Task Statistics**:
- **Total Tasks**: 73
- **Setup Tasks** (Phase 1): 3 tasks
- **Foundational Tasks** (Phase 2): 13 tasks (BLOCKING - must complete before stories)
- **US1 - Fix Book Crash** (P1): 6 tasks 🎯 MVP
- **US2 - User Signup** (P2): 10 tasks
- **US3 - User Login** (P2): 10 tasks
- **US4 - Header UI** (P3): 9 tasks
- **US5 - User Logout** (P3): 7 tasks
- **Polish & Validation** (Phase 8): 15 tasks

**Parallelization**: 28 tasks marked with [P] can run concurrently (38% parallelizable)

**Organization by User Story**:

Tasks organized into 8 phases following spec priorities:
1. **Phase 1 - Setup**: Environment config, DB project setup, secret generation (3 tasks)
2. **Phase 2 - Foundation**: Database schema, Better Auth config, API endpoints (13 tasks) - BLOCKING
3. **Phase 3 - US1 (P1)**: Fix book crash by uncommenting useDoc import, verify all docs render (6 tasks) - MVP MILESTONE
4. **Phase 4 - US2 (P2)**: Signup form with profile fields, backend signup logic, auth context integration (10 tasks)
5. **Phase 5 - US3 (P2)**: Login form verification, session persistence, auth context login (10 tasks)
6. **Phase 6 - US4 (P3)**: Navbar component registration, header UI enhancements, auth state display (9 tasks)
7. **Phase 7 - US5 (P3)**: Logout implementation, session cleanup, redirect logic (7 tasks)
8. **Phase 8 - Polish**: E2E validation, performance testing, documentation, code cleanup (15 tasks)

**Critical Path**:
```
Setup → Foundation → US1 (MVP)
                   ↓
        US2 + US3 (parallel)
                   ↓
        US4 + US5 (parallel)
                   ↓
              Polish
```

**MVP Scope (Minimum Viable Product)**:
- Phase 1 + Phase 2 + Phase 3 (US1) = 22 tasks
- Delivers: Book pages accessible, auth infrastructure ready
- Critical bug fixed, site restored to usable state

**Independent Test Criteria per Story**:

- **US1**: Navigate to `/docs/*` → page loads without error
- **US2**: Complete signup → account in DB → auto-login
- **US3**: Login → session cookie → refresh maintains auth
- **US4**: View header logged out → login → header updates → logout → reverts
- **US5**: Logout → session terminated → back button doesn't restore

**Story Dependencies**:
- US1 (P1): Must complete FIRST - blocks all site functionality
- US2 (P2) + US3 (P2): Can develop in parallel after Foundation
- US4 (P3): Depends on US2 + US3 (needs working auth)
- US5 (P3): Depends on US3 (needs login to test logout)

**Parallelization Examples**:

**Foundation Phase (After DB migration)**:
- Concurrent: Create lib/auth.ts, lib/auth-client.ts, verify dependencies
- Then: Update api-server.js, add /api/user/profile, create .env

**US2 + US3 Parallel Development**:
- US2 frontend (T023-T025) || US3 frontend (T033-T035)
- US2 backend (T027-T029) || US3 backend (T036-T038)
- US2 context (T030-T032) || US3 context (T039-T042)

**Key Implementation Details**:

1. **useDoc Fix** (US1):
   - Uncomment import in `src/theme/DocItem/index.jsx:2`
   - Import: `import { useDoc } from '@docusaurus/plugin-content-docs/client';`
   - Test all doc routes render without crash

2. **Database Schema**:
   - 4 Better Auth core tables (user, session, account, verification)
   - 1 custom extension (user_profiles with skill_level, backgrounds, learning_goal)
   - Migrations: 001_better_auth_core.sql, 002_user_profiles.sql

3. **Better Auth Config** (lib/auth.ts):
   - Neon pooled connection (max: 1 for serverless)
   - Cookie cache (5-min TTL, 80-90% query reduction)
   - Experimental joins enabled (2-3x performance)
   - Secure cookies (httpOnly, secure, sameSite=lax)

4. **Navbar Integration**:
   - Create ComponentTypes.js registering NavbarAuth as 'custom-Auth'
   - Uncomment docusaurus.config.js lines 81-84
   - Add skeleton loading state and user avatar

5. **Session Management**:
   - Cookie-based sessions with database persistence
   - 7-day session timeout (Better Auth default)
   - Cookie cache reduces DB calls by 80-90%
   - Survives serverless cold starts

**Testing Approach**:
- Manual E2E testing per specification (no automated tests)
- Test scenarios cover all 5 user stories + edge cases
- Performance targets: <3s cold start, <200ms cached, 80-90% cache hit rate

**Deployment Checklist** (Phase 8):
- Environment variables configured on Vercel
- Database migrations run on Neon DB
- BETTER_AUTH_URL updated to production domain
- Verify no personalization logic exists
- Test cold start latency and cache effectiveness

**Code Cleanup** (Phase 8):
- Remove duplicate Root.js (keep Root.tsx)
- Consolidate AuthContext.js/tsx
- Clean up unused auth/ directory files
- Document cookie cache strategy

## Outcome

- ✅ Impact: 73 actionable tasks organized by user story, enabling independent implementation and testing
- 🧪 Tests: Manual E2E testing specified - no automated test tasks generated per requirements
- 📁 Files: specs/007-auth-integration-fix/tasks.md created with complete task breakdown
- 🔁 Next prompts: `/sp.implement` to execute tasks in order, starting with MVP (Setup → Foundation → US1)
- 🧠 Reflection: Task organization by user story (P1/P2/P3 priorities) enables incremental delivery. MVP scope (22 tasks) delivers critical bug fix and auth foundation. 38% of tasks parallelizable for efficient execution. Clear dependencies and independent test criteria allow story-by-story validation.

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation followed spec priorities and plan architecture correctly
- Graders run and results (PASS/FAIL): Format validation PASS (all 73 tasks follow required checklist format), Story organization PASS (tasks grouped by US1-US5), MVP scope PASS (22 tasks for critical bug fix)
- Prompt variant (if applicable): N/A - standard /sp.tasks workflow
- Next experiment (smallest change to try): Consider adding time estimates per task or story for better sprint planning
