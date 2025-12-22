# Quickstart: Better Auth + Neon DB + Personalization

## Prerequisites
- Node.js v22 or higher
- A Neon DB account with a PostgreSQL database
- Better Auth secret key

## Environment Setup
1. Copy `.env.example` to `.env`
2. Update the `DATABASE_URL` with your Neon DB connection string
3. Set `BETTER_AUTH_SECRET_KEY` to a secure random value
4. Set `BETTER_AUTH_PUBLIC_KEY` if using email verification

## Running the Application
1. Install dependencies: `npm install`
2. Start the backend API server: `npm run start:api`
3. In a separate terminal, start the Docusaurus frontend: `npm run start`
4. Visit `http://localhost:3002` to access the application

## Key Features
- User registration with profile data (software_background, hardware_background, learning_goal)
- Login/logout functionality
- Chapter-level content personalization for authenticated users
- Profile management page accessible via the "Login" link in the navbar

## API Endpoints
- `/api/auth/*` - Better Auth endpoints (signup, login, logout, etc.)
- `/auth` - Authentication page (login/register)

## Troubleshooting
- If you see "useAuth must be used within an AuthProvider" error, ensure the AuthProvider is properly wrapped in Root.tsx
- If registration fails, check the browser console for errors and ensure all required fields are provided
- If Neon DB connection fails, verify the DATABASE_URL in your .env file