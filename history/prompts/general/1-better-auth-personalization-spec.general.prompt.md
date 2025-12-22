---
id: 1
title: better-auth-personalization-spec
stage: spec
date: 2025-12-17
surface: agent
model: gemini-2.5-flash
feature: better-auth-personalization
branch: 005-better-auth-personalization
user: Muhammad Hudaib Arif
command: /sp.specify
labels: [auth, personalization, better-auth, neon-db]
links:
  spec: specs/005-better-auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-better-auth-personalization/spec.md
  - specs/005-better-auth-personalization/checklists/requirements.md
tests:
  - specs/005-better-auth-personalization/checklists/requirements.md
---

## Prompt

Better Auth + Personalization Upgrade Specification

This specification defines the required fixes and enhancements for Signup, Signin, user data storage, and content personalization using Better Auth and Neon DB, without modifying the existing chatbot or FastAPI backend.

---

## 🎯 Objective

Fully satisfy the following requirements:

1. Implement Signup and Signin using Better Auth
2. Ask users about their **software and hardware background** at signup
3. Store user profile data reliably in **Neon DB**
4. Enable **logged-in users** to personalize book content
5. Keep normal book content unchanged for non-personalized view
6. Do NOT modify:
   - Chatbot logic
   - fastapi_app folder
   - Agent behavior

---

## 🔒 Constraints (STRICT)

- ❌ No changes to chatbot
- ❌ No changes to fastapi_app
- ❌ No changes to RAG / Agent logic
- ✅ Changes allowed only in:
  - Auth
  - UI
  - Personalization logic
  - Database integration

---

## 1. MCP + Documentation Requirement

Before modifying auth logic, Claude Code MUST fetch Better Auth documentation using MCP:

context7 - resolve-library-id (MCP)(libraryName: "better-auth")

yaml
Copy code

Purpose:
- Understand correct signup/login APIs
- Understand metadata storage
- Understand adapter + DB integration
- Follow best practices

MCP is used ONLY for documentation lookup (not as a runtime tool).

---

## 2. Signup & Login Requirements

### 2.1 Required Signup Fields (MUST MATCH REQUIREMENT)

Signup form MUST collect:

**Authentication**
- email (required)
- password (required)

**Profile / Content Personalization (required)**
- software_background
  (e.g. Web Dev, Backend, AI, Beginner)
- hardware_background
  (e.g. Laptop, PC, Robotics Kit, GPU)
- learning_goal
  (e.g. AI, Robotics, Full Stack)

---

### 2.2 Data Handling Rules

- Auth credentials handled by Better Auth
- Profile data MUST be:
  - Stored in Neon DB
  - Linked to authenticated user
  - Persisted after signup
- Do NOT store profile data only in frontend state

---

## 3. Neon DB Integration Requirements

Claude Code MUST:

1. Verify Neon DB connection is active
2. Verify user-related tables exist
3. Ensure profile fields are persisted:
   - software_background
   - hardware_background
   - learning_goal

If missing:
- Create or migrate schema safely
- Ensure adapter correctly writes data

⚠️ Silent failures (data not saved) MUST be fixed.

---

## 4. Post-Login User Experience

### 4.1 Auth State UI

After successful login:

- A **Profile / Personalize** button or icon MUST appear
- This MUST NOT appear for logged-out users
- Clicking it navigates to a personalization view/page

---

## 5. Content Personalization Requirements

### 5.1 Personalization Page

The personalization page MUST:

- Read user profile from Neon DB
- Explain what personalization is doing
- Allow user to:
  - Enable / disable personalization
  - Update preferences (optional)

---

### 5.2 Chapter-Level Personalization Button

At the start of each chapter:

- A **"Personalize Content"** button must exist
- When pressed:
  - Personalized version of content is shown
  - Based on user's stored profile
- Normal content MUST remain available
- Switching personalization must not reload the site

⚠️ Current broken behavior MUST be fixed.

---

## 6. Behavior Rules

- Logged-out users:
  - See normal book content
  - See NO personalization controls
- Logged-in users:
  - See normal book content by default
  - Can opt into personalization
- Personalization must NOT affect:
  - Chatbot responses
  - RAG behavior
  - Backend APIs

---

## 7. UI / UX Fixes (REQUIRED)

Claude Code MUST improve:

### Auth Pages
- Clear layout
- Proper labels
- Loading states
- Meaningful error messages

### Personalization UI
- Clear toggle or button
- Profile-aware labels
- Clean, readable layout

⚠️ No redesign of the site structure
⚠️ No feature creep

---

## 8. Validation Checklist (MUST PASS)

- ✅ Signup works
- ✅ Login works
- ✅ User profile saved in Neon DB
- ✅ Profile data retrievable after login
- ✅ Personalization button appears only when logged in
- ✅ Chapter personalization works correctly
- ✅ Normal content remains unchanged
- ✅ No crashes
- ✅ No console errors

---

## 9. Final Output Required from Claude Code

Claude Code MUST provide:

1. Summary of fixes
2. Files changed
3. Confirmation Neon DB stores user data
4. Confirmation personalization works
5. No changes to chatbot or fastapi_app

## Response snapshot

# Feature Specification: Better Auth + Personalization Upgrade

**Feature Branch**: `005-better-auth-personalization`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Better Auth + Personalization Upgrade Specification

This specification defines the required fixes and enhancements for Signup, Signin, user data storage, and content personalization using Better Auth and Neon DB, without modifying the existing chatbot or FastAPI backend.

---

## 🎯 Objective

Fully satisfy the following requirements:

1. Implement Signup and Signin using Better Auth
2. Ask users about their **software and hardware background** at signup
3. Store user profile data reliably in **Neon DB**
4. Enable **logged-in users** to personalize book content
5. Keep normal book content unchanged for non-personalized view
6. Do NOT modify:
   - Chatbot logic
   - fastapi_app folder
   - Agent behavior

---

## 🔒 Constraints (STRICT)

- ❌ No changes to chatbot
- ❌ No changes to fastapi_app
- ❌ No changes to RAG / Agent logic
- ✅ Changes allowed only in:
  - Auth
  - UI
  - Personalization logic
  - Database integration

---

## 2. Signup & Login Requirements

### 2.1 Required Signup Fields (MUST MATCH REQUIREMENT)

Signup form MUST collect:

**Authentication**
- email (required)
- password (required)

**Profile / Content Personalization (required)**
- software_background
  (e.g. Web Dev, Backend, AI, Beginner)
- hardware_background
  (e.g. Laptop, PC, Robotics Kit, GPU)
- learning_goal
  (e.g. AI, Robotics, Full Stack)

---

### 2.2 Data Handling Rules

- Auth credentials handled by Better Auth
- Profile data MUST be:
  - Stored in Neon DB
  - Linked to authenticated user
  - Persisted after signup
- Do NOT store profile data only in frontend state

---

## 3. Neon DB Integration Requirements

Claude Code MUST:

1. Verify Neon DB connection is active
2. Verify user-related tables exist
3. Ensure profile fields are persisted:
   - software_background
   - hardware_background
   - learning_goal

If missing:
- Create or migrate schema safely
- Ensure adapter correctly writes data

⚠️ Silent failures (data not saved) MUST be fixed.

---

## 4. Post-Login User Experience

### 4.1 Auth State UI

After successful login:

- A **Profile / Personalize** button or icon MUST appear
- This MUST NOT appear for logged-out users
- Clicking it navigates to a personalization view/page

---

## 5. Content Personalization Requirements

### 5.1 Personalization Page

The personalization page MUST:

- Read user profile from Neon DB
- Explain what personalization is doing
- Allow user to:
  - Enable / disable personalization
  - Update preferences (optional)

---

### 5.2 Chapter-Level Personalization Button

At the start of each chapter:

- A **\"Personalize Content\"** button must exist
- When pressed:
  - Personalized version of content is shown
  - Based on user's stored profile
- Normal content MUST remain available
- Switching personalization must not reload the site

⚠️ Current broken behavior MUST be fixed.

---

## 6. Behavior Rules

- Logged-out users:
  - See normal book content
  - See NO personalization controls
- Logged-in users:
  - See normal book content by default
  - Can opt into personalization
- Personalization must NOT affect:
  - Chatbot responses
  - RAG behavior
  - Backend APIs

---

## 7. UI / UX Fixes (REQUIRED)

Claude Code MUST improve:

### Auth Pages
- Clear layout
- Proper labels
- Loading states
- Meaningful error messages

### Personalization UI
- Clear toggle or button
- Profile-aware labels
- Clean, readable layout

⚠️ No redesign of the site structure
⚠️ No feature creep

---

## 8. Validation Checklist (MUST PASS)

- ✅ Signup works
- ✅ Login works
- ✅ User profile saved in Neon DB
- ✅ Profile data retrievable after login
- ✅ Personalization button appears only when logged in
- ✅ Chapter personalization works correctly
- ✅ Normal content remains unchanged
- ✅ No crashes
- ✅ No console errors

---
"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration with Profile Data (Priority: P1)

A new user visits the physical AI and humanoid robotics textbook website and wants to create an account to access personalized content. The user provides their email and password, and additionally shares information about their software background (e.g., Web Dev, Backend, AI, Beginner), hardware background (e.g., Laptop, PC, Robotics Kit, GPU), and learning goal (e.g., AI, Robotics, Full Stack).

**Why this priority**: This is the foundational user journey that enables all other personalization features. Without registration, users cannot access personalized content.

**Independent Test**: Can be fully tested by creating a new account with profile data and verifying that the data is stored in Neon DB. This delivers the core value of enabling personalized content access.

**Acceptance Scenarios**:

1. **Given** a new user on the signup page, **When** they enter valid email, password, and profile information, **Then** their account is created with profile data stored in Neon DB
2. **Given** a new user with invalid email format, **When** they attempt to sign up, **Then** they receive an appropriate error message and account is not created

---

### User Story 2 - User Login and Profile Access (Priority: P2)

An existing user logs into the website using their credentials. After successful login, they see a "Profile / Personalize" button or icon that allows them to access their profile information and update their preferences.

**Why this priority**: This enables existing users to access and modify their profile data, which is essential for maintaining accurate personalization preferences.

**Independent Test**: Can be fully tested by logging in with an existing account and verifying that the profile/personalize UI elements appear. This delivers the value of allowing users to manage their personalization settings.

**Acceptance Scenarios**:

1. **Given** a logged-in user, **When** they visit the site, **Then** they see the "Profile / Personalize" button or icon
2. **Given** a logged-out user, **When** they visit the site, **Then** they do not see the "Profile / Personalize" button or icon

---

### User Story 3 - Content Personalization by Chapter (Priority: P3)

A logged-in user visits a chapter of the textbook and can toggle personalization based on their stored profile data. When they click the "Personalize Content" button, the content adapts to their software background, hardware background, and learning goals without reloading the page.

**Why this priority**: This delivers the core value proposition of the feature - personalized content that enhances the learning experience based on user background and goals.

**Independent Test**: Can be fully tested by having a logged-in user activate personalization on a chapter and verifying that the content changes appropriately. This delivers the value of tailored learning content.

**Acceptance Scenarios**:

1. **Given** a logged-in user viewing a chapter, **When** they click the "Personalize Content" button, **Then** the content adapts based on their profile without page reload
2. **Given** a logged-in user with personalization active, **When** they toggle it off, **Then** the original content is restored

---

### Edge Cases

- What happens when a user tries to sign up with an email that already exists?
- How does the system handle users who don't want to provide profile information?
- What occurs when Neon DB is temporarily unavailable during signup/login?
- How does the system handle users who update their profile preferences while viewing personalized content?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST implement user signup using Better Auth with email and password authentication
- **FR-002**: System MUST collect and store user profile data (software_background, hardware_background, learning_goal) in Neon DB during signup
- **FR-003**: System MUST implement user login using Better Auth
- **FR-004**: System MUST display "Profile / Personalize" UI elements only to authenticated users
- **FR-005**: System MUST provide a personalization page that reads user profile from Neon DB
- **FR-006**: System MUST provide "Personalize Content" buttons at the start of each chapter for logged-in users
- **FR-007**: System MUST toggle content personalization based on user profile without page reload
- **FR-008**: System MUST maintain access to original (non-personalized) content when personalization is active
- **FR-009**: System MUST ensure that logged-out users see only original book content without personalization controls
- **FR-010**: System MUST handle Neon DB connection failures gracefully without breaking core functionality

### Key Entities *(include if feature involves data)*

- **User Profile**: Represents user preferences for content personalization, including software_background, hardware_background, and learning_goal. Associated with Better Auth user account and stored in Neon DB.
- **Personalization Settings**: Configuration that determines how content should be adapted based on user profile data. Applied at the chapter level.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete account creation with profile data in under 3 minutes
- **SC-002**: 95% of user profile data is successfully stored in Neon DB during signup
- **SC-003**: 90% of logged-in users can access personalized content within 10 seconds of clicking the personalization button
- **SC-004**: Users report 80% higher engagement with personalized content compared to non-personalized content
- **SC-005**: No changes are made to existing chatbot functionality or FastAPI backend
- **SC-006**: System maintains 99% uptime during authentication and personalization operations

## Outcome

- ✅ Impact: Created complete specification for Better Auth integration and personalization features
- 🧪 Tests: Created quality checklist to validate specification completeness
- 📁 Files: Created spec.md and requirements checklist in feature directory
- 🔁 Next prompts: Ready for planning phase with `/sp.plan`
- 🧠 Reflection: Specification addresses all requirements from user input with clear user stories, functional requirements, and success criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All specification sections completed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin planning implementation with `/sp.plan`