# Data Model: Book Platform Auth, DB & Personalization

**Feature**: 004-auth-db-personalization
**Created**: 2025-12-16
**Status**: Complete

## Entities

### User
**Description**: Represents a registered user with authentication credentials and background metadata

**Fields**:
- `id` (TEXT): Unique user identifier from Better Auth
- `email` (TEXT): User's email address (required)
- `skill_level` (TEXT): User's skill level (metadata)
- `hardware_background` (TEXT): User's hardware background (metadata)
- `learning_goal` (TEXT): User's learning goal (metadata)
- `created_at` (TIMESTAMP): Account creation timestamp
- `updated_at` (TIMESTAMP): Last update timestamp

**Validation Rules**:
- Email must be valid email format
- skill_level, hardware_background, learning_goal are optional during registration but recommended

**Relationships**:
- One-to-many with UserChapterState (user has many chapter personalization states)

### Chapter
**Description**: Represents a book chapter that can be personalized based on user background

**Fields**:
- `id` (TEXT): Unique chapter identifier
- `title` (TEXT): Chapter title
- `content` (TEXT): Original chapter content (immutable)
- `path` (TEXT): File path or URL to the chapter

**Validation Rules**:
- ID must be unique
- Title and content must not be empty

**Relationships**:
- Many-to-many with UserChapterState (chapter has many personalization states for different users)

### UserChapterState
**Description**: Stores the personalization state for a specific user-chapter combination

**Fields**:
- `user_id` (TEXT): Reference to User ID (foreign key)
- `chapter_id` (TEXT): Reference to Chapter ID
- `personalization_level` (TEXT): Level of personalization applied (e.g., beginner, intermediate, advanced)
- `language` (TEXT): Language preference for personalization
- `updated_at` (TIMESTAMP): Timestamp of last personalization update

**Validation Rules**:
- user_id and chapter_id combination must be unique
- personalization_level must be one of predefined values
- language must be valid language code

**Relationships**:
- Belongs to User (many UserChapterStates per User)
- Belongs to Chapter (many UserChapterStates per Chapter)

### PersonalizedContentCache (Optional)
**Description**: Cached version of personalized content for improved performance

**Fields**:
- `user_id` (TEXT): Reference to User ID
- `chapter_id` (TEXT): Reference to Chapter ID
- `content` (TEXT): Personalized content cache
- `updated_at` (TIMESTAMP): Timestamp of cache update

**Validation Rules**:
- user_id and chapter_id combination must be unique
- content must not exceed reasonable size limits

**Relationships**:
- Belongs to User (many cached contents per User)
- Belongs to Chapter (many cached contents per Chapter)

## Database Schema

### SQL Schema for Neon Postgres

```sql
-- User chapter state table
CREATE TABLE user_chapter_state (
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    personalization_level TEXT,
    language TEXT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (user_id, chapter_id)
);

-- Indexes for performance
CREATE INDEX idx_user_chapter_state_user_id ON user_chapter_state(user_id);
CREATE INDEX idx_user_chapter_state_chapter_id ON user_chapter_state(chapter_id);

-- Optional personalized content cache table
CREATE TABLE personalized_content_cache (
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    content TEXT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (user_id, chapter_id)
);

-- Indexes for performance
CREATE INDEX idx_personalized_content_cache_user_id ON personalized_content_cache(user_id);
CREATE INDEX idx_personalized_content_cache_chapter_id ON personalized_content_cache(chapter_id);
```

## State Transitions

### UserChapterState Lifecycle
1. **Created**: When user first personalizes a chapter
   - Set user_id, chapter_id, personalization_level, language
   - Set updated_at to current timestamp

2. **Updated**: When user modifies personalization settings
   - Update personalization_level and/or language
   - Update updated_at to current timestamp

3. **Retrieved**: When user revisits a chapter
   - Fetch existing personalization settings
   - Apply to content rendering

## Validation Rules Summary

- All required fields must be present
- Email format validation for user emails
- Unique constraints on user-chapter combinations
- Timestamps automatically managed
- Foreign key relationships maintained