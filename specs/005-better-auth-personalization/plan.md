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
