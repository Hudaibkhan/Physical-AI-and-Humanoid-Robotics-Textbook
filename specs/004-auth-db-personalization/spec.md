# Feature Specification: Book Platform Auth, DB & Personalization

**Feature Branch**: `004-auth-db-personalization`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "# Book Platform – Auth, DB & Personalization Specification (Non-Invasive)

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
- Zero changes to existing chatbot code"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - User Registration with Background Information (Priority: P1)

A new user visits the book platform and wants to create an account by providing their background information (skill level, hardware background, learning goal) during signup. The user should be able to register using Better Auth and have their background stored as metadata.

**Why this priority**: This is the foundational user journey that enables all other personalized features. Without user registration, no personalization can occur.

**Independent Test**: Can be fully tested by registering a new user with background information and verifying that the user account is created with the metadata stored appropriately.

**Acceptance Scenarios**:

1. **Given** a visitor on the registration page, **When** they fill in required auth details and background information, **Then** a new authenticated user account is created with background data stored as metadata
2. **Given** a user who has registered, **When** they log in again, **Then** their background information remains accessible to personalize their experience

---

### User Story 2 - Chapter Personalization Toggle (Priority: P1)

An authenticated user viewing a book chapter wants to customize the content to match their background (skill level, hardware background, learning goal). The user should see a "Personalize this chapter" button that applies tailored content based on their stored preferences.

**Why this priority**: This delivers the core value proposition of the feature - personalized book content based on user background.

**Independent Test**: Can be fully tested by having an authenticated user activate personalization on a chapter and seeing customized content based on their stored background information.

**Acceptance Scenarios**:

1. **Given** an authenticated user viewing a chapter page, **When** they click the "Personalize this chapter" button, **Then** the content adapts to match their background information (skill level, hardware background, learning goal)
2. **Given** a personalized chapter view, **When** the user changes their background information, **Then** subsequent personalizations reflect the updated preferences

---

### User Story 3 - Personalization State Persistence (Priority: P2)

An authenticated user wants their chapter personalization settings to be saved across sessions. The system should store and retrieve personalization preferences for each chapter in a database.

**Why this priority**: This enhances user experience by maintaining their preferences across sessions, reducing the need to repeatedly customize content.

**Independent Test**: Can be fully tested by having a user personalize a chapter, logging out, logging back in, and verifying the personalization settings are restored.

**Acceptance Scenarios**:

1. **Given** an authenticated user who has personalized a chapter, **When** they revisit the same chapter later, **Then** their personalization settings are automatically applied
2. **Given** a user with multiple personalized chapters, **When** they access any chapter, **Then** the appropriate personalization settings for that specific chapter are applied

---

### Edge Cases

- What happens when a user is not authenticated but clicks the personalization button?
- How does the system handle database connection failures when saving/retrieving personalization state?
- What occurs when a user updates their background information but has existing personalization settings?
- How does the system handle personalization for users who didn't provide background information during signup?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide Better Auth integration for user signup and signin functionality
- **FR-002**: System MUST collect user background information (skill_level, hardware_background, learning_goal) during registration
- **FR-003**: System MUST store user background information as auth metadata
- **FR-004**: System MUST provide a "Personalize this chapter" button on each chapter page
- **FR-005**: System MUST adapt chapter content based on user's background information when personalization is enabled
- **FR-006**: System MUST store personalization state in Neon Postgres database using the user_chapter_state table
- **FR-007**: System MUST save user_id, chapter_id, personalization_level, and language for each personalized chapter
- **FR-008**: System MUST retrieve and apply existing personalization settings when a user revisits a chapter
- **FR-009**: System MUST update personalization settings when users modify their background information
- **FR-010**: System MUST ensure personalized content remains grounded in the original chapter text
- **FR-011**: System MUST NOT modify existing chatbot, FastAPI backend, or book content files
- **FR-012**: System MUST use DATABASE_URL environment variable to connect to Neon Postgres

### Key Entities *(include if feature involves data)*

- **User**: Represents a registered user with authentication credentials and background metadata (skill_level, hardware_background, learning_goal)
- **Chapter**: Represents a book chapter that can be personalized based on user background
- **UserChapterState**: Stores the personalization state for a specific user-chapter combination (user_id, chapter_id, personalization_level, language, updated_at)
- **PersonalizedContentCache**: Optional cached version of personalized content for improved performance (user_id, chapter_id, content)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete account registration with background information in under 2 minutes
- **SC-002**: At least 70% of registered users engage with the personalization feature within their first week of registration
- **SC-003**: Personalized chapter content loads within 2 seconds of activation
- **SC-004**: The system maintains 99.5% uptime for authentication and personalization services
- **SC-005**: Users report a 40% improvement in content relevance compared to non-personalized experience
- **SC-006**: Database operations for storing and retrieving personalization state complete within 500ms
- **SC-007**: The existing chatbot functionality experiences zero downtime during the integration of personalization features