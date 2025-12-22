---
id: 1
title: better-auth-personalization-tasks
stage: tasks
date: 2025-12-17
surface: agent
model: gemini-2.5-flash
feature: better-auth-personalization
branch: 005-better-auth-personalization
user: Muhammad Hudaib Arif
command: /sp.tasks
labels: [auth, personalization, better-auth, neon-db, tasks]
links:
  spec: specs/005-better-auth-personalization/spec.md
  plan: specs/005-better-auth-personalization/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-better-auth-personalization/tasks.md
tests:
  - specs/005-better-auth-personalization/tasks.md
---

## Prompt

$ARGUMENTS

## Response snapshot

# Tasks: Better Auth + Neon DB + Personalization Fix

**Feature**: Better Auth + Neon DB + Personalization Fix
**Branch**: `005-better-auth-personalization`
**Generated**: 2025-12-17
**Input**: Feature specification from `/specs/005-better-auth-personalization/spec.md`

## Implementation Strategy

Implement Better Auth with Neon DB integration for user signup/login, collect user profile data (software_background, hardware_background, learning_goal) during registration, and enable content personalization for logged-in users. The approach follows an MVP-first strategy where User Story 1 (New User Registration) is implemented first to establish the foundational authentication system, followed by User Story 2 (Login and Profile Access), and finally User Story 3 (Content Personalization).

## Dependencies

- User Story 1 (Registration) must be completed before User Story 2 (Login) and User Story 3 (Personalization)
- User Story 2 (Login) must be completed before User Story 3 (Personalization)
- Neon DB setup must be completed before any auth functionality

## Parallel Execution Examples

Per User Story 1:
- [P] T006 [US1] Update auth.config.ts to use Neon DB adapter
- [P] T007 [US1] Update auth.config.ts to collect required profile fields
- [P] T008 [US1] Update auth page to collect required profile fields

Per User Story 2:
- [P] T020 [US2] Update AuthContext to use Better Auth session management
- [P] T021 [US2] Update Root.tsx to properly integrate AuthProvider
- [P] T022 [US2] Implement profile/personalize button visibility logic

Per User Story 3:
- [P] T030 [US3] Update PersonalizeChapterButton to use real API
- [P] T031 [US3] Implement chapter personalization logic
- [P] T032 [US3] Create personalization settings page

---

## Phase 1: Setup

- [ ] T001 Set up Neon DB adapter in Better Auth configuration
- [ ] T002 Verify Neon DB connection using DATABASE_URL from environment
- [ ] T003 Create/update database schema to include required profile fields
- [ ] T004 Update package.json with any necessary dependencies for Neon DB

## Phase 2: Foundational Tasks

- [ ] T005 Update Better Auth server configuration to use Neon DB instead of SQLite
- [ ] T006 [P] Update auth.config.ts to use Neon DB adapter
- [ ] T007 [P] Update auth.config.ts to collect required profile fields (software_background, hardware_background, learning_goal)
- [ ] T008 [P] Update Better Auth client configuration if needed
- [ ] T009 Fix "useAuth must be used within an AuthProvider" error by ensuring proper AuthProvider integration
- [ ] T010 Update api-server.js to properly integrate Better Auth with Express

## Phase 3: [US1] New User Registration with Profile Data

**Goal**: Enable new users to create accounts with profile data that gets stored in Neon DB

**Independent Test**: Can be fully tested by creating a new account with profile data and verifying that the data is stored in Neon DB. This delivers the core value of enabling personalized content access.

**Acceptance Scenarios**:
1. Given a new user on the signup page, When they enter valid email, password, and profile information, Then their account is created with profile data stored in Neon DB
2. Given a new user with invalid email format, When they attempt to sign up, Then they receive an appropriate error message and account is not created

- [ ] T011 [US1] Update signup form UI to collect software_background field
- [ ] T012 [US1] Update signup form UI to collect hardware_background field
- [ ] T013 [US1] Update signup form UI to collect learning_goal field
- [ ] T014 [US1] Add validation for required profile fields in signup form
- [ ] T015 [US1] Update signup API call to include profile fields
- [ ] T016 [US1] Test successful registration with profile data
- [ ] T017 [US1] Add proper error handling to signup form
- [ ] T018 [US1] Add loading states to signup form
- [ ] T019 [US1] Verify profile data is stored in Neon DB after registration

## Phase 4: [US2] User Login and Profile Access

**Goal**: Enable existing users to log in and access their profile information

**Independent Test**: Can be fully tested by logging in with an existing account and verifying that the profile/personalize UI elements appear. This delivers the value of allowing users to manage their personalization settings.

**Acceptance Scenarios**:
1. Given a logged-in user, When they visit the site, Then they see the "Profile / Personalize" button or icon
2. Given a logged-out user, When they visit the site, Then they do not see the "Profile / Personalize" button or icon

- [ ] T020 [US2] Update AuthContext to use Better Auth session management instead of manual token handling
- [ ] T021 [US2] Update Root.tsx to properly integrate AuthProvider with Docusaurus theme
- [ ] T022 [US2] Implement profile/personalize button visibility logic based on auth state
- [ ] T023 [US2] Update login form to work with Better Auth
- [ ] T024 [US2] Add proper error handling to login form
- [ ] T025 [US2] Add loading states to login form
- [ ] T026 [US2] Test successful login and profile data retrieval
- [ ] T027 [US2] Implement user profile page to display profile information
- [ ] T028 [US2] Add ability to update profile information from profile page
- [ ] T029 [US2] Verify profile data is retrieved correctly after login

## Phase 5: [US3] Content Personalization by Chapter

**Goal**: Enable logged-in users to toggle personalization based on their stored profile data

**Independent Test**: Can be fully tested by having a logged-in user activate personalization on a chapter and verifying that the content changes appropriately. This delivers the value of tailored learning content.

**Acceptance Scenarios**:
1. Given a logged-in user viewing a chapter, When they click the "Personalize Content" button, Then the content adapts based on their profile without page reload
2. Given a logged-in user with personalization active, When they toggle it off, Then the original content is restored

- [ ] T030 [US3] Update PersonalizeChapterButton to use real personalization API instead of mock implementation
- [ ] T031 [US3] Implement chapter personalization logic using user profile data
- [ ] T032 [US3] Create personalization settings page that reads user profile from Neon DB
- [ ] T033 [US3] Add "Personalize Content" button to chapter pages
- [ ] T034 [US3] Implement content switching logic without page reload
- [ ] T035 [US3] Ensure original content remains available when personalization is toggled
- [ ] T036 [US3] Add personalization explanation to personalization settings page
- [ ] T037 [US3] Test personalization toggle functionality
- [ ] T038 [US3] Verify personalization respects user profile data
- [ ] T039 [US3] Test that logged-out users don't see personalization controls

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T040 Add comprehensive error handling for Neon DB connection failures
- [ ] T041 Implement graceful degradation when DB is unavailable
- [ ] T042 Add proper loading states throughout auth flows
- [ ] T043 Improve UI/UX for auth pages (clear layout, proper labels, meaningful error messages)
- [ ] T044 Improve UI/UX for personalization components (clear toggle, profile-aware labels)
- [ ] T045 Verify no changes were made to chatbot or FastAPI backend
- [ ] T046 Test complete user flow: signup → login → personalization
- [ ] T047 Verify normal content remains unchanged for non-personalized view
- [ ] T048 Update documentation to reflect new auth and personalization features
- [ ] T049 Run final validation checklist: signup works, login works, profile saved in Neon DB, personalization button appears only when logged in, chapter personalization works correctly

## Outcome

- ✅ Impact: Created comprehensive task list for Better Auth + Neon DB + Personalization feature
- 🧪 Tests: Generated 49 specific, actionable tasks organized by user story with dependencies
- 📁 Files: Created tasks.md with structured phases and clear implementation path
- 🔁 Next prompts: Ready for implementation with `/sp.implement`
- 🧠 Reflection: Tasks follow MVP-first approach with User Story 1 as foundation, enabling incremental delivery

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All tasks follow required format with IDs, story labels, and file paths
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation with Phase 1 tasks