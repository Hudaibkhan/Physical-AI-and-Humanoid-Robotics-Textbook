# Quickstart Guide: Book Platform Auth, DB & Personalization

**Feature**: 004-auth-db-personalization
**Created**: 2025-12-16

## Prerequisites

Before starting the implementation, ensure the following are available:

### Environment Variables
```bash
# Better Auth Configuration
BETTER_AUTH_PUBLIC_KEY=your_public_key
BETTER_AUTH_SECRET_KEY=your_secret_key

# Database Configuration
DATABASE_URL=your_neon_postgres_connection_string

# LLM Configuration (for existing agent)
GEMINI_API_KEY=your_gemini_api_key
OPENAI_API_KEY=your_openai_api_key

# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
```

### Required Services
- Better Auth account and API keys
- Neon Postgres database instance
- Existing OpenAI Agents SDK setup
- Qdrant vector database access

## Setup Steps

### 1. Install Dependencies
```bash
npm install @better-auth/node @better-auth/react @neondatabase/serverless
# Add other required packages as needed
```

### 2. Database Setup
Run the following SQL to create required tables:

```sql
-- Create user chapter state table
CREATE TABLE user_chapter_state (
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    personalization_level TEXT,
    language TEXT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (user_id, chapter_id)
);

-- Create indexes for performance
CREATE INDEX idx_user_chapter_state_user_id ON user_chapter_state(user_id);
CREATE INDEX idx_user_chapter_state_chapter_id ON user_chapter_state(chapter_id);

-- Create optional personalized content cache table
CREATE TABLE personalized_content_cache (
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    content TEXT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (user_id, chapter_id)
);

-- Create indexes for performance
CREATE INDEX idx_personalized_content_cache_user_id ON personalized_content_cache(user_id);
CREATE INDEX idx_personalized_content_cache_chapter_id ON personalized_content_cache(chapter_id);
```

### 3. Configuration Files
Create or update your configuration files with the required environment variables.

## Implementation Sequence

### Phase 1: Authentication Setup
1. Configure Better Auth with custom metadata fields
2. Implement registration form with required fields:
   - Email (required)
   - Password (required)
   - Skill level (metadata)
   - Hardware background (metadata)
   - Learning goal (metadata)
3. Implement login form with email/password
4. Add auth state management to frontend

### Phase 2: Database Integration
1. Create database connection utilities
2. Implement CRUD operations for user_chapter_state
3. Add caching layer if needed

### Phase 3: Personalization Features
1. Add "Personalize this chapter" button to chapter pages
2. Create `/personalize` endpoint
3. Integrate with existing book_rag_agent to generate personalized content
4. Implement frontend display of personalized content

## Testing Checklist

- [ ] User registration with metadata collection works
- [ ] User login and session management works
- [ ] "Personalize this chapter" button appears on chapter pages
- [ ] Personalization API endpoint returns appropriate responses
- [ ] Database correctly stores and retrieves personalization state
- [ ] Existing chatbot functionality remains unchanged
- [ ] Personalized content maintains original citations and formatting
- [ ] Error handling works for unauthenticated users

## Deployment Notes

- Ensure environment variables are properly configured in deployment environment
- Database migrations should run before application startup
- Verify that existing chatbot endpoints are not affected
- Test personalization features with various user metadata combinations