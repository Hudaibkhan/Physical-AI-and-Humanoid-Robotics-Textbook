# Data Model: Better Auth + Personalization

## User Entity

### Core User Data (Managed by Better Auth)
- **id**: string - Unique user identifier
- **email**: string - User's email address (required, unique)
- **name**: string - User's name
- **createdAt**: Date - Account creation timestamp
- **updatedAt**: Date - Last update timestamp
- **emailVerified**: boolean - Whether email is verified

### Profile Data (Stored as User Metadata in Better Auth)
- **software_background**: string - User's software background (e.g., Web Dev, Backend, AI, Beginner)
- **hardware_background**: string - User's hardware background (e.g., Laptop, PC, Robotics Kit, GPU)
- **learning_goal**: string - User's learning goal (e.g., AI, Robotics, Full Stack)

## Personalization State (Optional - for tracking per-chapter settings)
- **user_id**: string - Reference to user
- **chapter_id**: string - Chapter identifier
- **personalization_level**: string - Level of personalization applied
- **language**: string - Preferred language
- **updated_at**: Date - Last updated timestamp

## Validation Rules
- Email must be valid email format
- Password must be at least 8 characters
- software_background, hardware_background, and learning_goal are required at signup
- User profile data must be persisted in Neon DB via Better Auth adapter

## State Transitions
- Unauthenticated → Pending Registration → Registered (with profile) → Personalization Enabled
- Each transition preserves user profile data in Neon DB