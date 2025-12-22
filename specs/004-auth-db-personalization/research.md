# Research Findings: Book Platform Auth, DB & Personalization

**Feature**: 004-auth-db-personalization
**Created**: 2025-12-16
**Status**: Complete

## MCP Context7 Documentation Research

### Decision: MCP Integration Approach
**Rationale**: Based on the user's plan requirement to "Call MCP context7 to fetch OpenAI Agents SDK documentation" and the project constitution, MCP tools must be used appropriately for personalization operations. However, the user specifically stated "MCP must NOT be used for personalization, auth, or database logic", which means MCP tools should only be used for their intended purposes (chunking, embeddings, Qdrant operations) and not for authentication or database operations.

**Alternatives considered**:
- Direct database operations vs MCP database tools
- MCP for all operations vs selective MCP usage
- Chose selective MCP usage per specification requirements

## Project Structure Analysis

### Decision: Frontend Integration Points
**Rationale**: Need to identify where the "Personalize this chapter" button should be added. Based on the existing project structure, the book content is likely served through the Docusaurus-based documentation site with ChatKit integration.

**Research findings**:
- Book content likely exists in markdown files within the documentation structure
- Frontend is likely built with React components that can be extended
- Existing chatbot widget should remain untouched per requirements

**Alternatives considered**:
- Modifying existing chatbot vs adding separate personalization UI
- Server-side vs client-side personalization rendering
- Chose client-side approach to maintain separation from chatbot

## Better Auth Integration Research

### Decision: Better Auth Implementation Strategy
**Rationale**: Better Auth needs to be integrated with specific signup fields for metadata collection. Based on the user requirements, the signup form must include email, password, and three metadata fields (skill_level, hardware_background, learning_goal).

**Research findings**:
- Better Auth supports custom user metadata through the `userMetadata` field
- Can be integrated with React frontend and FastAPI backend
- Supports social login providers if needed in the future

**Alternatives considered**:
- Custom auth vs Better Auth
- Different metadata storage approaches
- Chose Better Auth for its robust feature set and ease of use

## Database Schema Research

### Decision: Neon Postgres Schema Design
**Rationale**: Need to implement the exact schema specified in the user requirements with the correct fields and types.

**Research findings**:
- Neon Postgres supports all required field types (TEXT, TIMESTAMP)
- Need to create indexes on user_id and chapter_id for performance
- Proper timestamp handling with timezone awareness

**Alternatives considered**:
- Different field types vs TEXT for flexibility
- Different timestamp formats
- Chose specified schema exactly as required

## Existing Book Content Structure

### Decision: Content Personalization Approach
**Rationale**: Need to understand how existing book content is structured to implement personalization while keeping it grounded in original text.

**Research findings**:
- Content is likely in markdown format with potential citations
- Existing RAG functionality should be leveraged with user context
- Personalization should modify presentation, not core content meaning

**Alternatives considered**:
- Content rewriting vs presentation modification
- Server-side vs client-side personalization
- Chose server-side personalization using existing agent with user metadata