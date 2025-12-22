-- Migration: Create tables for personalization state

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