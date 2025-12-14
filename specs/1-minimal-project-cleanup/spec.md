# Feature Specification: Minimal Project Cleanup

**Feature Branch**: `1-minimal-project-cleanup`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "MINIMAL PROJECT CLEANUP SPECIFICATION

CONTEXT
The Book RAG Agent is working correctly.
Cleanup is required only to remove clutter and duplicate folders.
This is NOT a production deployment step.

GOAL
- Make the repository clean and readable
- Remove unused / duplicate files
- Keep the project fully runnable after cleanup

IMPORTANT RULES

1. Backend Safety Rule
- fastapi_app/ is the active backend
- DO NOT delete or modify ANY file inside fastapi_app/

2. Duplicate Backend
- Delete the entire backend/ folder if it exists

3. Environment Files
- DO NOT delete:
  - .env
  - .env.example (if present)

4. Remove Unused Files
Delete:
- test_*.py
- *_test.py
- debug files
- playground or demo scripts
- notebooks (.ipynb)
- unused markdown, json, or txt files
- commented-out old implementations

5. Frontend Cleanup
- Keep ChatKit integration files
- Remove unused / experimental frontend files only

6. Code Safety
- Do NOT change working logic
- Do NOT break imports
- Only delete clearly unused files

7. Validation
After cleanup:
- Run the project
- Confirm FastAPI app starts successfully
- Confirm agent initializes without errors

OUTPUT REQUIRED
- List deleted files
- Show final directory tree
- Confirm project runs successfully"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Repository Cleanup (Priority: P1)

As a developer, I want to remove unused and duplicate files from the repository so that the codebase is cleaner, more organized, and easier to navigate.

**Why this priority**: This is the core value proposition - removing clutter makes the project more maintainable and understandable for anyone working on it.

**Independent Test**: Can be fully tested by running the cleanup process and verifying that the project still functions correctly after removing specified files.

**Acceptance Scenarios**:

1. **Given** a repository with duplicate and unused files, **When** the cleanup process is executed, **Then** specified files are removed while keeping the project functional
2. **Given** a repository with test files and debug scripts, **When** the cleanup process is executed, **Then** test files and debug scripts are removed according to the rules

---

### User Story 2 - Safe Backend Preservation (Priority: P1)

As a developer, I want to ensure that the active backend (fastapi_app/) remains untouched during cleanup so that the working application continues to function.

**Why this priority**: Protecting the working backend is critical to maintaining functionality during cleanup.

**Independent Test**: Can be fully tested by verifying that files inside fastapi_app/ remain unchanged after cleanup.

**Acceptance Scenarios**:

1. **Given** a repository with fastapi_app/ as the active backend, **When** the cleanup process is executed, **Then** no files inside fastapi_app/ are modified or deleted

---

### User Story 3 - Validation Confirmation (Priority: P2)

As a developer, I want to confirm that the project runs successfully after cleanup so that I can be confident the cleanup didn't break anything.

**Why this priority**: Ensuring functionality after cleanup is essential to validate the success of the cleanup process.

**Independent Test**: Can be fully tested by attempting to start the FastAPI application and confirming the agent initializes correctly.

**Acceptance Scenarios**:

1. **Given** a cleaned repository, **When** the project is started, **Then** the FastAPI app starts successfully and the agent initializes without errors

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST remove all files matching the pattern test_*.py
- **FR-002**: System MUST remove all files matching the pattern *_test.py
- **FR-003**: System MUST remove all notebook files with extension .ipynb
- **FR-004**: System MUST remove the entire backend/ folder if it exists as a duplicate
- **FR-005**: System MUST preserve the fastapi_app/ folder and all its contents
- **FR-006**: System MUST preserve .env and .env.example files
- **FR-007**: System MUST remove debug files and playground scripts
- **FR-008**: System MUST provide a list of deleted files after cleanup
- **FR-009**: System MUST confirm the project runs successfully after cleanup
- **FR-010**: System MUST preserve ChatKit integration files during frontend cleanup

### Key Entities *(include if feature involves data)*

- **Repository Files**: Collection of files and directories in the codebase that need to be categorized as either to be kept or removed
- **FastAPI Application**: The active backend application that must remain functional after cleanup

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Repository contains 20% fewer files after cleanup while maintaining full functionality
- **SC-002**: FastAPI app starts successfully after cleanup with no initialization errors
- **SC-003**: Agent initializes without errors after cleanup
- **SC-004**: At least 10 unused files are removed from the repository
- **SC-005**: No files inside fastapi_app/ are modified or deleted during cleanup