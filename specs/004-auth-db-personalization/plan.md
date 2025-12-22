# Implementation Plan: Book Platform Auth, DB & Personalization

**Feature Spec**: [specs/004-auth-db-personalization/spec.md](../specs/004-auth-db-personalization/spec.md)
**Created**: 2025-12-16
**Status**: Draft
**Branch**: 004-auth-db-personalization

## Technical Context

This plan implements authentication and personalization features for the book platform using Better Auth and Neon Postgres, while maintaining strict separation from existing chatbot functionality.

**Key Technologies**:
- Better Auth for authentication
- Neon Postgres for personalization state
- Existing book_rag_agent for content personalization

**Known Unknowns**:
- None (all research completed in research.md)

**Dependencies**:
- Better Auth SDK installation and configuration
- Neon Postgres database setup
- Environment variables (BETTER_AUTH_PUBLIC_KEY, BETTER_AUTH_SECRET_KEY, DATABASE_URL)

## Constitution Check

This implementation must comply with the project constitution:

✅ **Agent-Driven Architecture**: Personalization will use existing book_rag_agent with user metadata
✅ **MCP Integration & Validation**: Will use MCP tools for any required operations
✅ **Production-Ready Implementation**: Code will be deployable and self-contained
✅ **Agent Response Quality**: Personalized content will maintain citation standards
✅ **System Consistency**: Personalization will maintain existing formatting and UX

**Gates**:
- [ ] No modifications to fastapi_app/ or chatbot-related folders
- [ ] Existing chatbot behavior remains unchanged
- [ ] MCP tools used appropriately for personalization operations

## Phase 0: Research & Analysis

### 0.1 MCP Context7 Documentation Research
- Research MCP context7 documentation for OpenAI Agents SDK integration
- Understand how to properly call MCP tools from agents
- Ensure compliance with context7 guidance for skill implementations

### 0.2 Project Structure Analysis
- Map out current frontend structure to identify integration points
- Locate existing book content files and their format
- Identify where "Personalize this chapter" button should be added

### 0.3 Better Auth Integration Research
- Research Better Auth best practices for React/Next.js applications
- Understand how to store user metadata (skill_level, hardware_background, learning_goal)
- Identify required signup form fields and validation

## Phase 1: Data Model & API Design

### 1.1 Database Schema Implementation
- Create SQL schema for `user_chapter_state` table
- Create optional `personalized_content_cache` table
- Define CRUD operations for personalization state
- Implement connection pooling with DATABASE_URL

### 1.2 API Contract Design
- Design `/personalize` endpoint contract
- Define request/response schema for personalization
- Specify authentication token validation requirements
- Document error handling patterns

### 1.3 Data Model Documentation
- Document User entity with metadata fields
- Document Chapter entity with personalization options
- Document UserChapterState with relationships
- Define validation rules for all entities

## Phase 2: Authentication Implementation

### 2.1 Backend Auth Setup
- Configure Better Auth with required environment variables
- Implement user metadata storage during registration
- Create authentication middleware for protected routes

### 2.2 Frontend Auth Integration
- Implement signup form with required fields:
  - Email (required)
  - Password (required)
  - Skill level (metadata)
  - Hardware background (metadata)
  - Learning goal (metadata)
- Implement signin form with email/password
- Add auth state management to frontend

## Phase 3: Personalization Implementation

### 3.1 Chapter Personalization UI
- Add "Personalize this chapter" button to chapter pages
- Implement UI state management for personalization
- Create loading states and error handling

### 3.2 Backend Personalization Logic
- Create `/personalization` endpoint
- Implement database operations for state persistence
- Integrate with existing book_rag_agent with user metadata
- Ensure personalized content maintains original citations

### 3.3 Frontend Integration
- Connect personalization button to backend endpoint
- Display personalized content while preserving formatting
- Implement caching for improved performance

## Phase 4: Testing & Validation

### 4.1 Authentication Testing
- Test user registration with metadata collection
- Test login/logout functionality
- Verify metadata persistence and retrieval

### 4.2 Personalization Testing
- Test chapter personalization functionality
- Verify database state persistence
- Test personalization accuracy based on user metadata

### 4.3 Integration Testing
- Ensure chatbot functionality remains unchanged
- Verify no breaking changes to existing code
- Test error handling for unauthenticated users

## Phase 5: Deployment Preparation

### 5.1 Environment Configuration
- Document required environment variables
- Create sample .env files
- Define database migration procedures

### 5.2 Production Readiness
- Optimize database queries
- Implement proper error logging
- Add performance monitoring