import { createAuthClient } from 'better-auth/client';

// Get auth backend URL from environment variable
const AUTH_BACKEND_URL = process.env.NEXT_PUBLIC_AUTH_BACKEND_URL || 'http://localhost:8000';

// Create the Better Auth client
export const client = createAuthClient({
  baseURL: AUTH_BACKEND_URL,
});

// Export the client for use in components
export const { signIn, signOut, useSession, signUp } = client;