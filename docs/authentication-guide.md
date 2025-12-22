# Better Auth + Neon DB + Personalization Guide for Docusaurus

This document explains how to use and extend the authentication and personalization features in the Physical AI and Humanoid Robotics Textbook platform.

## Overview

The authentication and personalization system consists of:

1. **Backend API Server**: An Express.js server that handles Better Auth API routes and personalization
2. **Frontend Docusaurus App**: Uses Better Auth client to communicate with the backend
3. **Database**: Neon PostgreSQL database for storing user data and sessions

## Authentication Features

### User Registration
When registering, users are required to provide:
- **Email**: Valid email address for account access
- **Password**: Minimum 8 characters
- **Software Background**: Select from options like Web Development, Backend Development, AI/ML, Beginner, Intermediate, Advanced
- **Hardware Background**: Text field for hardware experience (e.g., Laptop, PC, Robotics Kit, GPU)
- **Learning Goal**: Text field for learning objectives (e.g., AI, Robotics, Full Stack)

### User Login
- Use the "Login" button in the top right corner
- After successful login, a dropdown menu appears with "Account" and "Personalization" options

### Account Management
- Access your account information via the dropdown menu after login
- View your profile details (email, software background, hardware background, learning goal)
- Update your profile information

## Personalization Features

### Profile-Based Personalization
The system uses your profile information to tailor content to your background and goals:
- Content examples are adapted to your hardware/software background
- Complexity level is adjusted based on your experience
- Learning paths are customized to help you achieve your goals

### Chapter Personalization
- On chapter pages, you'll see a "Personalize this chapter" button (when logged in)
- Click the button to adapt the content to your profile
- The original content remains available when you toggle personalization off

### Personalization Settings
- Access personalization settings from the dropdown menu after login
- Update your profile information to refine personalization
- Learn how the personalization system works

## Backend Setup

### 1. API Server (`api-server.js`)

The backend server is built with Express.js and integrates Better Auth using the Node.js integration:

```javascript
const express = require('express');
const { toNodeHandler } = require('better-auth/integrations/node');
const { auth } = require('./src/auth/auth.config');

// Convert Better Auth handler to Express-compatible middleware
const betterAuthHandler = toNodeHandler(auth);

app.use('/api/auth', async (req, res) => {
  const request = new Request(`${req.protocol}://${req.get('host')}${req.originalUrl}`, {
    method: req.method,
    headers: new Headers(req.headers),
    body: req.method !== 'GET' && req.method !== 'HEAD' ? JSON.stringify(req.body) : undefined,
  });

  try {
    const response = await betterAuthHandler(request);

    for (const [key, value] of response.headers) {
      res.setHeader(key, value);
    }

    res.status(response.status).send(await response.text());
  } catch (error) {
    console.error('Better Auth error:', error);
    // Check if it's a database connection error
    if (error.message && (error.message.includes('database') || error.message.includes('connection') || error.message.includes('pool'))) {
      res.status(503).json({ error: 'Database service temporarily unavailable. Please try again later.' });
    } else {
      res.status(500).json({ error: 'Authentication service error' });
    }
  }
});

// Personalization endpoint
app.post('/api/personalize', async (req, res) => {
  try {
    const { chapter_id, user_metadata } = req.body;

    if (!chapter_id || !user_metadata) {
      return res.status(400).json({ error: 'Missing required fields: chapter_id and user_metadata' });
    }

    // In a real implementation, this would use AI to personalize content based on user profile
    // For now, we'll return a basic personalized version
    const personalizedContent = `**PERSONALIZED CONTENT**\n\n${req.body.currentContent || 'Sample chapter content'}\n\n*This content has been tailored to your background: ${user_metadata.software_background || 'general'} software background, with focus on ${user_metadata.hardware_background || 'general robotics'} concepts, to help you achieve your ${user_metadata.learning_goal || 'learning goals'}.*`;

    res.json({
      personalized_content: personalizedContent,
      chapter_id: chapter_id,
      user_metadata: user_metadata
    });
  } catch (error) {
    console.error('Personalization error:', error);
    // Check if it's a database connection error
    if (error.message && (error.message.includes('database') || error.message.includes('connection') || error.message.includes('pool'))) {
      res.status(503).json({ error: 'Database service temporarily unavailable. Please try again later.' });
    } else {
      res.status(500).json({ error: 'Personalization processing failed' });
    }
  }
});
```

### 2. Auth Configuration (`src/auth/auth.config.ts`)

The authentication configuration defines how Better Auth should work with Neon DB:

```typescript
import { betterAuth } from "better-auth";
import { postgresAdapter } from "@better-auth/adapter-postgres";
import { Pool } from "pg";

// Initialize PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  max: 20, // Maximum number of clients in the pool
  idleTimeoutMillis: 30000, // Close idle clients after 30 seconds
  connectionTimeoutMillis: 2000, // Return an error after 2 seconds if connection could not be established
});

export const auth = betterAuth({
  secret: process.env.BETTER_AUTH_SECRET_KEY || "your-secret-key-change-in-production",
  app: {
    name: "Book Platform",
  },
  database: {
    provider: "postgresql", // Switched to PostgreSQL for Neon DB
    url: process.env.DATABASE_URL || "",
    // Use PostgreSQL adapter for Neon DB
    adapter: postgresAdapter(pool, {
      user: {
        // Define the table name for users if needed
        table: "auth_user",
      },
      account: {
        // Define the table name for accounts if needed
        table: "auth_account",
      },
      session: {
        // Define the table name for sessions if needed
        table: "auth_session",
      },
    }),
  },
  socialProviders: {
    // Add social providers if needed in the future
  },
  user: {
    // Add custom fields for user metadata
    additionalFields: {
      software_background: {
        type: "string",
        required: true, // Make required as per spec
        defaultValue: "",
      },
      hardware_background: {
        type: "string",
        required: true, // Make required as per spec
        defaultValue: "",
      },
      learning_goal: {
        type: "string",
        required: true, // Make required as per spec
        defaultValue: "",
      },
    },
  },
  account: {
    // Account-related configuration
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
  },
});
```

## Frontend Setup

### 1. Better Auth Client (`src/auth/client.ts`)

Creates the Better Auth client for frontend use:

```typescript
import { createAuthClient } from 'better-auth/client';

// Create the Better Auth client
// Note: Better Auth client will automatically detect the API base URL from the auth configuration
export const client = createAuthClient({
  // The client will automatically use the same base URL as the server when possible
  // In development, this should match your API server URL
});

// Export the client for use in components
export const { signIn, signOut, useSession, signUp } = client;
```

### 2. Authentication Context (`src/auth/context/AuthContext.tsx`)

Manages authentication state in the React application with Better Auth session management:

```typescript
// Login function
const login = async (email: string, password: string) => {
  dispatch({ type: 'LOGIN_START' });

  try {
    const { signIn } = await import('../client');
    const response = await signIn.email({ email, password });

    if (response?.session) {
      // Better Auth handles session automatically, but we can store additional data if needed
      const user = {
        id: response.session.user.id,
        email: response.session.user.email,
        software_background: response.session.user.$?.software_background || '',
        hardware_background: response.session.user.$?.hardware_background || '',
        learning_goal: response.session.user.$?.learning_goal || '',
        created_at: new Date(response.session.user.createdAt),
        updated_at: new Date(response.session.user.updatedAt),
      };

      dispatch({ type: 'LOGIN_SUCCESS', payload: { user } });
    } else {
      dispatch({ type: 'LOGIN_FAILURE' });
      throw new Error('Login failed');
    }
  } catch (error) {
    dispatch({ type: 'LOGIN_FAILURE' });
    throw error;
  }
};
```

### 3. Auth Provider (`src/theme/Root.tsx`)

Wraps the entire Docusaurus app with the authentication provider:

```javascript
import React from 'react';
import { AuthProvider } from '../auth/context/AuthContext';

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}
```

## API Routes

Better Auth automatically creates the following API endpoints:

- `POST /api/auth/sign-in/email` - Email/password login
- `POST /api/auth/sign-up/email` - Email/password registration
- `POST /api/auth/sign-out` - Logout
- `GET /api/auth/session` - Get current session
- `POST /api/auth/change-password` - Change password
- Additional routes for password reset, email verification, etc.

Plus custom personalization endpoint:
- `POST /api/personalize` - Personalize content based on user profile

## Environment Variables

Create a `.env` file in the project root:

```env
BETTER_AUTH_SECRET_KEY=your-super-secret-key-here
DATABASE_URL=postgresql://neondb_owner:npg_SCBmkvLR9Zc3@ep-red-recipe-a1vni1nn-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
BETTER_AUTH_PUBLIC_KEY=1a92984e095c395517127a49aa68fd0e7cd3c4e9a3315e0ae2dd1c9a12bfb16b
```

## Running the Application

1. Start the API server:
   ```bash
   npm run start:api
   ```

2. In a separate terminal, start the Docusaurus development server:
   ```bash
   npm run start
   ```

## Key Features

- **Secure Authentication**: Password-based authentication with secure session management
- **Neon DB Integration**: PostgreSQL database via Neon for reliable data storage
- **Custom User Fields**: Extended user profiles with software background, hardware background, and learning goals
- **Session Management**: Automatic session handling with configurable expiration
- **Content Personalization**: Tailored content based on user profile information
- **TypeScript Support**: Full TypeScript integration for type safety

## Security Considerations

- Always use HTTPS in production
- Keep the `BETTER_AUTH_SECRET_KEY` secure and never commit to version control
- Configure proper CORS settings for production
- Regularly update dependencies
- Use strong, unique passwords

## Troubleshooting

### Authentication Issues
If authentication isn't working:
1. Ensure the API server is running on the correct port (default: 8000)
2. Verify that the `DATABASE_URL` in environment variables is correct
3. Check that the Neon DB connection string is properly formatted
4. Confirm that CORS allows requests from your Docusaurus frontend

### Registration Issues
- Ensure all required fields are filled out
- Verify your email address is valid
- Check that your password meets the minimum length requirement

### Personalization Not Working
- Ensure you're logged in
- Verify your profile information is complete
- Check that JavaScript is enabled in your browser