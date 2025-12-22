import { createAuthClient } from 'better-auth/client';
import ENV_CONFIG from '../config/env.config';

// Get auth backend URL from environment configuration
const AUTH_BACKEND_URL = ENV_CONFIG.AUTH_BACKEND_URL;

// Create the Better Auth client
export const client = createAuthClient({
  baseURL: AUTH_BACKEND_URL,
});

// Export the client for use in components
export const { signIn, signOut, useSession, signUp } = client;