# Tasks: Auth Integration & Book Crash Fix

**Input**: Design documents from `/specs/007-auth-integration-fix/`
**Prerequisites**: plan.md (✅ complete), spec.md (✅ complete), research.md (✅ complete)

**Tests**: No automated test tasks included per specification - manual E2E testing specified

**Organization**: Tasks are grouped by user story (P1, P2, P3) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=P1, US2=P2, US3=P2, US4=P3, US5=P3)
- Include exact file paths in descriptions

## Path Conventions

- **Web application structure**: Docusaurus frontend + Express backend
- Frontend: `src/` (React/Docusaurus components)
- Backend: `lib/` (Better Auth config), `api-server.js` (Express)
- Database: `migrations/` (SQL migration files)
- Config: Root-level config files (`.env`, `docusaurus.config.js`)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Environment configuration and database initialization

- [x] T001 Create `.env.example` file at repository root with DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, FRONTEND_URL, NODE_ENV template
- [x] T002 [P] Create `migrations/` directory at repository root for SQL migration files
- [x] T003 Generate BETTER_AUTH_SECRET using `node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"` and document in setup guide

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Setup

- [ ] T004 Setup Neon DB project and obtain pooled connection string (endpoint with `-pooler` suffix)
- [ ] T005 Create Better Auth core schema migration in `migrations/001_better_auth_core.sql` with user, session, account, verification tables per research.md
- [ ] T006 [P] Create custom user_profiles extension in `migrations/002_user_profiles.sql` with id, userId, skill_level, software_background, hardware_background, learning_goal, createdAt, updatedAt
- [ ] T007 Run Better Auth CLI migration: `npx @better-auth/cli migrate` to create core tables
- [ ] T008 Execute custom migration for user_profiles table against Neon DB

### Better Auth Backend Configuration

- [ ] T009 Create `lib/` directory at repository root if it doesn't exist
- [ ] T010 [P] Create `lib/auth.ts` implementing Better Auth configuration with Pool(DATABASE_URL), cookie cache (5-min TTL), experimental joins enabled, secure cookies, per research.md Section 1
- [ ] T011 [P] Create `lib/auth-client.ts` implementing Better Auth React client with baseURL configuration for frontend use
- [ ] T012 Update `api-server.js` to import Better Auth handler and mount at `/api/auth/*` route with proper CORS (credentials: true)
- [ ] T013 Add `/api/user/profile` GET endpoint in `api-server.js` to fetch user_profiles data joined with user (requires authentication)

### Environment & Dependencies Verification

- [ ] T014 Verify `better-auth@1.4.7`, `pg@8.11.3`, `express@5.2.1` are in package.json dependencies
- [ ] T015 Create `.env` file (gitignored) with actual values for DATABASE_URL (Neon pooled), BETTER_AUTH_SECRET (generated), BETTER_AUTH_URL (http://localhost:3002 for dev), FRONTEND_URL, NODE_ENV=development
- [ ] T016 Test database connection by starting api-server.js and checking console logs for successful Pool initialization

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Fix Book Rendering Crash (Priority: P1) 🎯 MVP

**Goal**: Restore access to all book pages by fixing the `useDoc is not defined` error

**Independent Test**: Navigate to any book page (e.g., `/docs/module-1/introduction`), verify page loads without crash, content displays correctly, and Docusaurus hooks function properly

### Implementation for User Story 1

- [x] T017 [US1] Locate `src/theme/DocItem/index.jsx` and identify the commented-out useDoc import on line 2
- [x] T018 [US1] Uncomment and verify import statement: `import { useDoc } from '@docusaurus/plugin-content-docs/client';` in `src/theme/DocItem/index.jsx`
- [x] T019 [US1] Start Docusaurus dev server: `npm run start` and navigate to multiple doc pages (/docs/*, /modules/*) to verify all render without errors
- [x] T020 [US1] Test navigation between doc pages to ensure no crashes when switching between documents
- [x] T021 [US1] Verify personalization button still appears at bottom of doc pages (existing functionality preserved)
- [x] T022 [US1] Delete unused duplicate file `src/theme/Root.js` to avoid confusion (keep Root.tsx)

**Checkpoint**: At this point, all book pages should be accessible and functional. This is the MVP - site is usable again.

---

## Phase 4: User Story 2 - User Signup (Priority: P2)

**Goal**: Enable new users to create accounts with profile information collection

**Independent Test**: Navigate to `/signup`, fill in name/email/password plus optional profile fields, submit form, verify account created in Neon DB, automatic login occurs, and welcome confirmation displays

### Frontend Signup Form Enhancement

- [ ] T023 [P] [US2] Update `src/pages/signup.jsx` to add profile fields: Skill Level (dropdown: Beginner/Intermediate/Advanced), Software Background (textarea), Hardware Background (textarea), Learning Goal (textarea) with clear labels
- [ ] T024 [P] [US2] Update `src/auth/components/RegistrationForm.tsx` to include same profile fields with proper TypeScript types
- [ ] T025 [US2] Add form validation in signup components for: email format, password min 8 chars, required fields (name, email, password), optional fields max lengths (500 chars for text areas)
- [ ] T026 [US2] Update signup form submission to call Better Auth sign-up endpoint with profile data: `POST /api/auth/sign-up/email` with body {name, email, password, skill_level, software_background, hardware_background, learning_goal}

### Backend Signup Logic

- [ ] T027 [US2] Extend Better Auth signup handler in `lib/auth.ts` to capture additional profile fields from request body
- [ ] T028 [US2] Implement post-signup hook in `lib/auth.ts` that inserts user profile data into user_profiles table after Better Auth creates user in core tables
- [ ] T029 [US2] Add error handling in signup flow for: duplicate email (unique constraint violation), database connection failures, validation errors with user-friendly messages per FR-019

### Auth Context Update

- [ ] T030 [US2] Update `src/auth/context/AuthContext.js` register function to accept profile parameters and call Better Auth client signup
- [ ] T031 [US2] Ensure register function stores returned session token and dispatches REGISTER_SUCCESS action with user data
- [ ] T032 [US2] Delete unused duplicate `src/auth/context/AuthContext.tsx` or consolidate with .js version

**Checkpoint**: New users can successfully sign up with profile data, accounts persist in Neon DB, and auto-login works

---

## Phase 5: User Story 3 - User Login (Priority: P2)

**Goal**: Enable returning users to authenticate and establish sessions

**Independent Test**: Navigate to `/login`, enter valid email/password, submit, verify authentication succeeds, session cookie set, redirect to main page, and session persists on page refresh

### Frontend Login Form Verification

- [ ] T033 [P] [US3] Verify `src/pages/login.jsx` has email and password fields with proper validation (email format, password min length)
- [ ] T034 [P] [US3] Verify `src/auth/components/LoginForm.tsx` correctly calls Better Auth client sign-in method
- [ ] T035 [US3] Add error handling in login form for: invalid credentials (show generic error without revealing which field), network errors, database connection errors

### Backend Login Logic

- [ ] T036 [US3] Verify Better Auth login handler at `/api/auth/sign-in/email` is mounted correctly in `api-server.js`
- [ ] T037 [US3] Test login endpoint returns session cookie (robotics-auth.session_token) with httpOnly, secure, sameSite=lax attributes
- [ ] T038 [US3] Verify cookie cache is working by checking that second request within 5 minutes doesn't hit database (check server logs)

### Auth Context Login Integration

- [ ] T039 [US3] Update `src/auth/context/AuthContext.js` login function to call Better Auth client signIn.email() method
- [ ] T040 [US3] Ensure login function stores session data and dispatches LOGIN_SUCCESS action
- [ ] T041 [US3] Implement fetchUser function to call `/api/auth/get-session` on app mount to restore session from cookie
- [ ] T042 [US3] Add loading state management during session restoration to prevent UI flash

**Checkpoint**: Users can log in, sessions persist across page refreshes, and session cookies work correctly with Better Auth

---

## Phase 6: User Story 4 - Header UI Authentication State (Priority: P3)

**Goal**: Display appropriate header navigation based on authentication status

**Independent Test**: View header while logged out (shows Login/Signup), log in and verify header updates to show Profile/Logout, log out and verify header reverts to Login/Signup

### Navbar Component Registration

- [ ] T043 [US4] Create `src/theme/NavbarItem/ComponentTypes.js` importing ComponentTypes from @theme-original and NavbarAuth component
- [ ] T044 [US4] Register NavbarAuth as 'custom-Auth' type in ComponentTypes export object
- [ ] T045 [US4] Update `docusaurus.config.js` navbar items array: uncomment lines 81-84 to add {type: 'custom-Auth', position: 'right'}

### NavbarAuth Component Enhancement

- [ ] T046 [P] [US4] Update `src/components/NavbarAuth.js` to replace "Loading..." text with skeleton UI (animated placeholder with 80px width, 32px height, gray background)
- [ ] T047 [P] [US4] Add user avatar circle to authenticated state in NavbarAuth.js displaying first letter of email in uppercase with gradient background
- [ ] T048 [US4] Verify NavbarAuth dropdown menu includes: Account link (/auth), Personalization link (/personalization-settings), Logout button with onClick handler
- [ ] T049 [US4] Test header updates immediately on login/logout by checking React Context subscription triggers re-render

### Styling & UX Polish

- [ ] T050 [P] [US4] Add CSS animations for dropdown menu transitions in NavbarAuth.js
- [ ] T051 [P] [US4] Ensure header layout doesn't shift when toggling between logged-in/logged-out states (consistent button widths)

**Checkpoint**: Header dynamically displays correct UI elements based on authentication state, with smooth transitions

---

## Phase 7: User Story 5 - User Logout (Priority: P3)

**Goal**: Enable users to securely terminate their sessions

**Independent Test**: While logged in, click Logout button in header, verify session terminates, cookie cleared, user redirected to home page, and subsequent navigation shows unauthenticated state

### Logout Implementation

- [ ] T052 [US5] Verify Better Auth logout handler is mounted at `/api/auth/sign-out` in `api-server.js`
- [ ] T053 [US5] Update `src/auth/context/AuthContext.js` logout function to call Better Auth client signOut() and dispatch LOGOUT action
- [ ] T054 [US5] Ensure logout function clears cookie on client and server, removes localStorage authToken if present
- [ ] T055 [US5] Add redirect logic after logout to navigate user to home page (/) or login page (/login)

### Session Cleanup

- [ ] T056 [P] [US5] Verify logout deletes session from Neon DB sessions table
- [ ] T057 [P] [US5] Test that attempting to access authenticated features after logout prompts re-login
- [ ] T058 [US5] Verify browser back button after logout doesn't restore authenticated state (session is truly terminated)

**Checkpoint**: Users can securely log out, sessions are fully terminated, and re-authentication is required for protected features

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, documentation, and deployment preparation

### End-to-End Validation

- [ ] T059 Complete manual E2E test: Signup new user → logout → login → navigate docs → logout → verify all flows work
- [ ] T060 Test database connection failure handling by temporarily providing invalid DATABASE_URL and verifying user-friendly error messages appear
- [ ] T061 Test concurrent signup attempts with same email to verify unique constraint prevents duplicates and shows error message
- [ ] T062 Verify session persistence by: login → close browser → reopen → verify still logged in (within 7-day session timeout)

### Performance Testing

- [ ] T063 Measure cold start latency on Vercel by deploying to preview environment and testing first request after idle period (target: <3s)
- [ ] T064 Verify cookie cache effectiveness by checking server logs show reduced database queries after first session validation (target: 80-90% cache hit rate)
- [ ] T065 Test concurrent user load (10+ simultaneous signups/logins) to verify Neon pooled connection handles load

### Documentation & Deployment

- [ ] T066 [P] Document environment variables setup in README.md including Neon DB connection string format and BETTER_AUTH_SECRET generation command
- [ ] T067 [P] Create deployment checklist for Vercel: environment variables, database migrations run, Better Auth URL updated to production domain
- [ ] T068 Verify no personalization logic exists in codebase (search for personalization functions that modify content based on user profile - should only collect data, not use it)
- [ ] T069 [P] Add comments to user_profiles table creation explaining fields are for future personalization (out of scope for this phase)

### Code Cleanup

- [ ] T070 [P] Remove or consolidate duplicate auth context files (AuthContext.js vs AuthContext.tsx)
- [ ] T071 [P] Remove unused Root.js file (keep Root.tsx only)
- [ ] T072 [P] Review and clean up auth/ directory files that are no longer needed after Better Auth integration
- [ ] T073 Add code comments to lib/auth.ts explaining cookie cache strategy and session configuration choices

---

## Task Summary

**Total Tasks**: 73
**Breakdown by User Story**:
- Setup (Phase 1): 3 tasks
- Foundational (Phase 2): 13 tasks (BLOCKING)
- US1 - Fix Book Crash (P1): 6 tasks 🎯 MVP
- US2 - User Signup (P2): 10 tasks
- US3 - User Login (P2): 10 tasks
- US4 - Header UI (P3): 9 tasks
- US5 - User Logout (P3): 7 tasks
- Polish (Phase 8): 15 tasks

**Parallelization Opportunities**: 28 tasks marked with [P] can run concurrently once dependencies are met

---

## Dependencies & Execution Order

### Critical Path (Sequential)

```
Phase 1 (Setup) → Phase 2 (Foundation) → Phase 3 (US1 - MVP)
                                        ↓
                          Phase 4 (US2) + Phase 5 (US3) [Parallel]
                                        ↓
                          Phase 6 (US4) + Phase 7 (US5) [Parallel]
                                        ↓
                                  Phase 8 (Polish)
```

### Story Dependencies

- **US1 (P1) - Book Crash Fix**: Must complete FIRST - blocks site functionality
- **US2 (P2) - Signup** and **US3 (P2) - Login**: Can be developed in parallel after Foundation complete
- **US4 (P3) - Header UI**: Depends on US2 and US3 (needs auth state to display)
- **US5 (P3) - Logout**: Depends on US3 (needs login to test logout)

### Independent Test Verification Per Story

- **US1**: Navigate to `/docs/*` → page loads without error
- **US2**: Complete signup form → account in DB → auto-login
- **US3**: Login with credentials → session cookie set → page refresh maintains auth
- **US4**: View header logged out → login → header updates → logout → header reverts
- **US5**: Click logout → session terminated → back button doesn't restore auth

---

## Parallel Execution Examples

### Phase 2 Foundation (After T008 DB migration complete)

**Concurrent Batch 1**:
- T010 (Create lib/auth.ts)
- T011 (Create lib/auth-client.ts)
- T014 (Verify package.json dependencies)

**Concurrent Batch 2** (after T010-T011 complete):
- T012 (Update api-server.js with Better Auth handler)
- T013 (Add /api/user/profile endpoint)
- T015 (Create .env file)

### Phase 4 + Phase 5 (US2 + US3 in parallel)

**US2 Signup Tasks** (T023-T032):
- T023, T024, T025 (frontend forms) can run in parallel
- T027, T028, T029 (backend logic) can run in parallel after T010-T013 complete
- T030, T031, T032 (auth context) sequential after backend ready

**US3 Login Tasks** (T033-T042):
- T033, T034, T035 (frontend verification) can run in parallel
- T036, T037, T038 (backend verification) can run in parallel
- T039, T040, T041, T042 (auth context) sequential after backend ready

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Phase 1 + Phase 2 + Phase 3 (US1)** = MVP

This delivers:
✅ Book pages accessible again (critical bug fixed)
✅ Database and auth infrastructure ready
✅ Foundation for auth features

**Estimated MVP Completion**: 22 tasks (T001-T022)

### Incremental Delivery

1. **Sprint 1**: MVP (US1) - Restore site functionality
2. **Sprint 2**: US2 + US3 - Core authentication (signup + login)
3. **Sprint 3**: US4 + US5 - UX polish (header + logout)
4. **Sprint 4**: Phase 8 - Testing, performance, deployment

### Testing Approach

**Manual E2E Testing per specification** - No automated test suite required

**Test Scenarios** (reference spec.md acceptance scenarios):
1. Book page access (US1)
2. Signup flow with profile fields (US2)
3. Login flow with session persistence (US3)
4. Header state transitions (US4)
5. Logout and session termination (US5)
6. Edge cases: duplicate email, invalid credentials, DB failures, session expiry

---

## Format Validation ✅

**Checklist Format Compliance**: All 73 tasks follow required format:
- ✅ Checkbox prefix: `- [ ]`
- ✅ Task ID: T001-T073 sequential
- ✅ [P] markers: 28 parallelizable tasks identified
- ✅ [Story] labels: US1-US5 labels applied to user story phases
- ✅ File paths: Specific paths included in task descriptions
- ✅ Execution order: Tasks ordered by dependencies and story priorities

**Ready for implementation** ✅
