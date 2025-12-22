# Research: Better Auth + Neon DB + Personalization Fix

## Decision: Update Better Auth to use Neon DB instead of SQLite
**Rationale**: The current auth.config.ts uses SQLite but the project has a Neon DB connection string in .env. Need to switch to Neon DB to meet the requirement of storing profile data reliably in Neon DB.

## Decision: Fix Better Auth signup to collect the required fields
**Rationale**: The current implementation collects `skill_level` but the spec requires `software_background`, `hardware_background`, and `learning_goal`. Need to update the additionalFields in the auth config.

## Decision: Update AuthContext to use Better Auth's built-in session management
**Rationale**: The current AuthContext implementation manually manages tokens in localStorage, but Better Auth handles session management automatically. This may be causing the "useAuth must be used within an AuthProvider" error.

## Decision: Fix the personalization flow to use real API instead of mock implementation
**Rationale**: The PersonalizeChapterButton.tsx currently has a mock implementation that just adds text. Need to implement real personalization API calls that use user profile data.

## Decision: Add proper error handling and loading states to auth forms
**Rationale**: The current auth pages need better UX with proper error messages and loading states to address the "Registration failed" issue mentioned in the plan.

## Decision: Verify and fix the AuthProvider integration with Docusaurus
**Rationale**: The "useAuth must be used within an AuthProvider" error suggests there may be issues with how the AuthProvider is integrated with the Docusaurus theme system.