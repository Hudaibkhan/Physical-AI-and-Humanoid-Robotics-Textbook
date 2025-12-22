# Implementation Plan: Auth Integration & Book Crash Fix

**Branch**: `007-auth-integration-fix` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-auth-integration-fix/spec.md`

## Summary

Fix critical `useDoc` crash blocking all book pages and implement secure authentication system using Better Auth + Neon DB. System will support signup/login/logout with user profile collection (skill level, backgrounds, learning goals) while explicitly excluding personalization logic in this phase. Backend deployed on Vercel with session management via cookie-cached database sessions.

**Critical Path**: P1 bug fix (book crash) must complete before P2 auth implementation to restore site functionality.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 22.x), React 18.0.0
**Primary Dependencies**: Docusaurus 3.0.0, Better Auth 1.4.7, pg 8.11.3, Express 5.2.1
**Storage**: Neon DB (Postgres-compatible) with pooled connections
**Testing**: Manual E2E testing (login/signup/logout flows)
**Target Platform**: Vercel (serverless functions), Neon Cloud (database)
**Project Type**: Web application (Docusaurus frontend + Express backend)
**Performance Goals**: <3s cold start, <200ms cached requests, 80-90% cache hit rate
**Constraints**: No personalization logic, no chatbot changes, no FastAPI modifications
**Scale/Scope**: 5 user stories, 33 functional requirements, 4 DB tables + 1 custom extension

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Agent Architecture**: ✅ PASS (Not applicable - auth system separate from RAG agent)
**MCP Integration**: ✅ PASS (Not applicable - no MCP tools involved)
**Production-Ready Implementation**: ✅ PASS (Better Auth is production-ready, Neon DB is production-grade)
**Output Rules**: ✅ PASS (No agent outputs involved; standard HTTP auth responses)
**Quality Rules**: ✅ PASS (Following Spec-Kit Plus workflow: specify → clarify → plan → tasks → implement)
**Validation Requirements**: ⚠️ DEFERRED (Validation checklist created during implementation phase)

**Project Consistency**: ✅ PASS
- Project uses Docusaurus 3.0.0 (standard React-based documentation framework)
- Better Auth integrates cleanly without modifying existing RAG architecture
- Neon DB used for auth data only; does not conflict with Qdrant vector storage

## Project Structure

### Documentation (this feature)

```text
specs/007-auth-integration-fix/
├── plan.md              # This file
├── research.md          # Phase 0 output (✅ completed)
├── data-model.md        # Phase 1 output (⬇️ next)
├── quickstart.md        # Phase 1 output (⬇️ next)
├── contracts/           # Phase 1 output (⬇️ next)
│   └── auth-api.yml     # OpenAPI spec for auth endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (Docusaurus + Express backend)
├── lib/
│   ├── auth.ts                      # Better Auth server config (NEW)
│   └── auth-client.ts               # Better Auth React client (NEW)
│
├── api-server.js                    # Express server (EXISTS - MODIFY)
│
├── src/
│   ├── theme/
│   │   ├── Root.tsx                 # Global auth wrapper (EXISTS - NO CHANGE NEEDED)
│   │   ├── Root.js                  # Unused duplicate (DELETE)
│   │   ├── DocItem/
│   │   │   ├── index.jsx            # Broken useDoc import (EXISTS - FIX IMPORT)
│   │   │   └── Layout/
│   │   │       └── index.js         # Working useDoc usage (EXISTS - NO CHANGE)
│   │   └── NavbarItem/
│   │       └── ComponentTypes.js    # Custom navbar registry (NEW)
│   │
│   ├── auth/
│   │   ├── context/
│   │   │   ├── AuthContext.js       # Existing context (EXISTS - UPDATE for Better Auth)
│   │   │   └── AuthContext.tsx      # Unused duplicate (DELETE or CONSOLIDATE)
│   │   └── ...                      # Other auth files (REVIEW/CLEANUP)
│   │
│   ├── components/
│   │   ├── NavbarAuth.js            # Auth navbar component (EXISTS - ENHANCE)
│   │   ├── LoginForm.tsx            # Login UI (EXISTS - VERIFY)
│   │   └── SignupForm.tsx           # Signup UI (EXISTS - ADD PROFILE FIELDS)
│   │
│   └── pages/
│       ├── login.jsx                # Login page (EXISTS - VERIFY)
│       └── signup.jsx               # Signup page (EXISTS - ADD PROFILE FIELDS)
│
├── migrations/
│   └── 001_user_profiles.sql        # Custom schema extension (NEW)
│
├── docusaurus.config.js             # Docusaurus config (EXISTS - UNCOMMENT navbar)
│
├── .env                             # Environment variables (NEW)
└── .env.example                     # Environment template (NEW)
```

**Structure Decision**: Web application pattern selected. Docusaurus frontend communicates with Express backend via `/api/auth/*` routes. Better Auth handles session management with Neon DB persistence. No changes to existing FastAPI app (`fastapi_app/` directory) per explicit requirements.

## Complexity Tracking

> This section tracks violations of constitution principles that require justification.

**No violations identified.** This feature integrates cleanly with existing architecture:
- Authentication is separate from RAG agent system
- Better Auth is a well-established library (not custom implementation)
- Uses standard Postgres adapter (no custom database abstractions)
- Follows Docusaurus recommended patterns for global providers

---

# Phase 0: Outline & Research ✅ COMPLETE

**Status**: ✅ **COMPLETED** - See [research.md](./research.md)

### Key Research Findings

1. **Better Auth Integration**: Use v1.4.7 with Neon pooled connection (`-pooler` endpoint), cookie cache enabled (5-min TTL)
2. **Database Schema**: 4 core Better Auth tables + 1 custom `user_profiles` extension table
3. **useDoc Crash Fix**: Uncomment import from `@docusaurus/plugin-content-docs/client` in `src/theme/DocItem/index.jsx:2`
4. **Navbar Integration**: Register custom navbar type via `ComponentTypes.js` (90% already implemented)
5. **Session Persistence**: Cookie-based cache + database sessions reduces DB queries by 80-90%

---

# Phase 1: Design & Contracts

**Status**: ⬇️ **IN PROGRESS**

## Data Model Design

See [data-model.md](./data-model.md) for complete entity definitions.

### Entity Overview

| Entity | Purpose | Storage | Managed By |
|--------|---------|---------|------------|
| User | Core user identity | Neon DB (`user` table) | Better Auth |
| Session | Auth session tracking | Neon DB (`session` table) + Cookie Cache | Better Auth |
| Account | Password/OAuth credentials | Neon DB (`account` table) | Better Auth |
| UserProfile | Extended profile data | Neon DB (`user_profiles` table) | Custom (this app) |

### Key Relationships
- `User` 1:N `Session` (one user can have multiple active sessions)
- `User` 1:1 `UserProfile` (every user has one profile extension)
- `User` 1:N `Account` (user can have multiple auth providers - email/password, OAuth, etc.)

## API Contracts

See [contracts/auth-api.yml](./contracts/auth-api.yml) for OpenAPI specification.

### Endpoint Summary

| Method | Endpoint | Purpose | Request | Response |
|--------|----------|---------|---------|----------|
| POST | `/api/auth/sign-up/email` | User signup | `{email, password, name, profile}` | `{user, session}` |
| POST | `/api/auth/sign-in/email` | User login | `{email, password}` | `{user, session}` |
| POST | `/api/auth/sign-out` | User logout | (cookie-based) | `{success: true}` |
| GET | `/api/auth/get-session` | Get current session | (cookie-based) | `{user, session}` or null |
| GET | `/api/user/profile` | Get extended profile | (auth required) | `{profile}` |
| PATCH | `/api/user/profile` | Update profile | `{profile}` | `{profile}` (out of scope) |

### Authentication Flow

```
┌─────────┐                    ┌──────────────┐                    ┌──────────┐
│ Browser │                    │ Express/Auth │                    │ Neon DB  │
└────┬────┘                    └──────┬───────┘                    └────┬─────┘
     │                                │                                  │
     │ POST /api/auth/sign-up/email  │                                  │
     │───────────────────────────────>│                                  │
     │  {email, password, name}       │  INSERT INTO user, account,      │
     │                                │  user_profiles                    │
     │                                │─────────────────────────────────>│
     │                                │                                  │
     │                                │  CREATE session                  │
     │                                │─────────────────────────────────>│
     │                                │                                  │
     │  Set-Cookie: session_token     │                                  │
     │<───────────────────────────────│                                  │
     │  {user, session}               │                                  │
     │                                │                                  │
     │ GET /dashboard                 │                                  │
     │ Cookie: session_token          │                                  │
     │───────────────────────────────>│                                  │
     │                                │  VALIDATE session (or cache)     │
     │                                │─────────────────────────────────>│
     │                                │                                  │
     │  200 OK (authenticated)        │                                  │
     │<───────────────────────────────│                                  │
     │                                │                                  │
     │ POST /api/auth/sign-out        │                                  │
     │───────────────────────────────>│                                  │
     │                                │  DELETE session                  │
     │                                │─────────────────────────────────>│
     │  Clear-Cookie: session_token   │                                  │
     │<───────────────────────────────│                                  │
```

## Component Architecture

### Frontend Components

```
Root.tsx (Global wrapper)
  └─ AuthProvider (React Context)
      ├─ Navbar
      │   └─ ComponentTypes → custom-Auth
      │       └─ NavbarAuth.js
      │           ├─ (Loading) → Skeleton UI
      │           ├─ (Authenticated) → Dropdown with profile/logout
      │           └─ (Unauthenticated) → Login/Signup buttons
      │
      ├─ Layout
      │   └─ DocRoot
      │       └─ DocProvider (Docusaurus auto-wrap)
      │           └─ DocItemLayout
      │               └─ useDoc() ✅ Safe to use here
      │
      └─ Pages
          ├─ /login → LoginForm.tsx
          └─ /signup → SignupForm.tsx
              ├─ Name (required)
              ├─ Email (required)
              ├─ Password (required)
              ├─ Skill Level (optional)
              ├─ Software Background (optional)
              ├─ Hardware Background (optional)
              └─ Learning Goal (optional)
```

### Backend Architecture

```
Express Server (api-server.js)
  ├─ CORS middleware (credentials: true)
  ├─ JSON body parser
  │
  ├─ /api/auth/* → Better Auth handler
  │   ├─ auth.ts (Better Auth config)
  │   │   ├─ Database: Pool (Neon -pooler)
  │   │   ├─ Session: Cookie cache (5-min TTL)
  │   │   └─ Secret: BETTER_AUTH_SECRET
  │   │
  │   └─ Built-in endpoints:
  │       ├─ POST /sign-up/email
  │       ├─ POST /sign-in/email
  │       ├─ POST /sign-out
  │       └─ GET /get-session
  │
  └─ /api/user/* → Custom user profile endpoints
      └─ GET /profile (fetch extended profile)
```

## Quickstart Guide

See [quickstart.md](./quickstart.md) for complete developer setup instructions.

---

## Phase 1 Summary

✅ Data model defined (4 Better Auth tables + 1 custom extension)
✅ API contracts specified (6 endpoints documented)
✅ Component architecture designed (frontend + backend)
✅ Authentication flows diagrammed
⬇️ **Next**: Generate data-model.md, contracts/auth-api.yml, quickstart.md

---

## Re-evaluation: Constitution Check Post-Design

**Agent Architecture**: ✅ PASS - Auth system remains separate from OpenAI Agents SDK RAG system
**Production-Ready Implementation**: ✅ PASS - Better Auth + Neon DB are production-grade solutions
**System Consistency**: ✅ PASS - No conflicts with existing RAG architecture or MCP tools

**No new violations introduced during design phase.**

---

## Implementation Phases (from Spec)

These phases will be broken down into specific tasks during `/sp.tasks` command:

### Phase 1: Diagnose & Fix Book Rendering Crash (P1)
- Identify all `useDoc` usages and confirm import issue
- Fix import statement in `src/theme/DocItem/index.jsx`
- Test all doc pages render without crash
- Validate Docusaurus context hooks work correctly

### Phase 2: Implement Better Auth Backend (P2)
- Set up Neon DB and run Better Auth migrations
- Create custom `user_profiles` table
- Configure Better Auth in `lib/auth.ts`
- Integrate with Express server in `api-server.js`
- Test auth endpoints with Postman/curl

### Phase 3: Neon DB Integration (P2)
- Initialize Neon pooled connection
- Verify environment variables loaded correctly
- Test database connection from backend
- Implement custom profile creation on signup

### Phase 4: Frontend Signup/Login UI (P2)
- Update SignupForm.tsx to include profile fields
- Verify LoginForm.tsx works with Better Auth
- Test form validation and error handling
- Ensure session established on successful auth

### Phase 5: Header UI State Management (P3)
- Create `ComponentTypes.js` to register custom navbar type
- Uncomment navbar config in `docusaurus.config.js`
- Enhance NavbarAuth.js loading states
- Test header updates on login/logout

### Phase 6: Final Validation & Testing (P3)
- End-to-end signup flow test
- End-to-end login/logout flow test
- Verify session persistence across page refreshes
- Test cold start performance
- Verify no personalization logic present

---

## Dependencies & Prerequisites

### External Services
- **Neon DB**: Active project with pooled connection string
- **Vercel**: Deployment platform with environment variable support
- **Better Auth**: npm package (already installed v1.4.7)

### Internal Dependencies
- **Docusaurus 3.0.0**: Must remain functional during auth integration
- **Existing AuthContext**: Needs update to use Better Auth instead of custom implementation
- **NavbarAuth component**: Already built, needs ComponentTypes registration

### Technical Dependencies
- Node.js 22.x runtime
- Better Auth CLI for migrations (`npx @better-auth/cli migrate`)
- Environment variable management (`.env` files)

---

## Risk Summary (from research.md)

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| useDoc fails after import fix | Low | High | Restructure to use DocItemLayout |
| Neon connection pool exhaustion | Medium | Medium | Use -pooler endpoint, set max: 1 |
| Session loss on cold starts | Low | Medium | Enable cookie cache (5-min TTL) |
| Auth breaks Docusaurus context | Very Low | High | Already tested - Root.tsx placement correct |

---

## Next Steps

1. ✅ **Phase 0 Complete**: Research findings documented in [research.md](./research.md)
2. ⬇️ **Phase 1 In Progress**: Generate detailed artifacts:
   - [data-model.md](./data-model.md) - Entity definitions and schemas
   - [contracts/auth-api.yml](./contracts/auth-api.yml) - OpenAPI specification
   - [quickstart.md](./quickstart.md) - Developer setup guide
3. ⏳ **Phase 2 Pending**: Run `/sp.tasks` to generate implementation tasks
4. ⏳ **Phase 3 Pending**: Run `/sp.implement` to execute tasks

---

## Appendix: Environment Variables

```bash
# .env.example (template for developers)

# Database Configuration (Neon)
DATABASE_URL=postgresql://user:password@ep-xxx-pooler.us-east-2.aws.neon.tech/dbname?sslmode=require

# Better Auth Configuration
BETTER_AUTH_SECRET=<GENERATE_32_CHAR_RANDOM_STRING>
BETTER_AUTH_URL=http://localhost:3002

# Frontend Configuration
FRONTEND_URL=http://localhost:3002

# Node Environment
NODE_ENV=development
```

**Secret Generation:**
```bash
# Generate BETTER_AUTH_SECRET
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"
```

---

**Plan Status**: Phase 0 ✅ Complete | Phase 1 ⬇️ In Progress
**Next Command**: Continue with data-model.md, contracts/, and quickstart.md generation
