import { createAuthClient } from 'better-auth/client';

// Create the Better Auth client
export const client = createAuthClient({
  baseURL: 'https://fastapi-backend-for-book.vercel.app',  // API server URL
});

// Export the client for use in components
export const { signIn, signOut, useSession, signUp } = client;