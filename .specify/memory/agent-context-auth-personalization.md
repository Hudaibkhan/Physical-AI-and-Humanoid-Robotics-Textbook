# Agent Context: Auth, DB & Personalization Technologies

## Technologies Added for Feature 004-auth-db-personalization

### Authentication
- Better Auth: Modern authentication solution with user metadata support
- User metadata fields: skill_level, hardware_background, learning_goal
- Registration flow: email, password + 3 metadata fields
- Session management and token validation

### Database
- Neon Postgres: Cloud-native Postgres database
- Schema: user_chapter_state and personalized_content_cache tables
- Connection: via DATABASE_URL environment variable
- Operations: CRUD for personalization state

### Personalization
- Chapter-level personalization based on user metadata
- Integration with existing book_rag_agent
- Personalized content maintains original citations
- State persistence across sessions

### API Endpoints
- /auth/register: User registration with metadata
- /auth/login: User authentication
- /personalize: Generate personalized chapter content
- /personalization-state/{chapter_id}: Manage personalization settings

## Constraints
- No modifications to fastapi_app/ or chatbot-related folders
- Existing chatbot behavior remains unchanged
- MCP tools NOT used for auth or database logic
- Personalized content stays grounded in original text