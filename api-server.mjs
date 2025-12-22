// api-server.js - Node.js server with Express to handle Better Auth and API routes
import './load-env.js';  // Load environment variables before other imports
import express from 'express';
import cors from 'cors';
import cookieParser from 'cookie-parser';
import { auth } from './lib/auth.mjs';
import { toNodeHandler } from 'better-auth/node';
import { Pool } from 'pg';

const app = express();
const port = process.env.PORT || 8000;

// CORS middleware with credentials enabled (required for cookies)
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3002',
  credentials: true,  // CRITICAL: Allow cookies to be sent
  methods: ['GET', 'POST', 'PATCH', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization']
}));

app.use(express.json());
app.use(cookieParser());  // Parse cookies for session management

// Convert Better Auth handler to Express-compatible middleware
const betterAuthHandler = toNodeHandler(auth);

app.use('/api/auth', betterAuthHandler);

// Health check endpoint
app.get('/', (req, res) => {
  res.json({ message: 'Book RAG Agent API with Better Auth is running' });
});

// Get user profile endpoint (requires authentication)
// Returns user data merged with profile data
// Auto-creates profile record for existing users without one (backward compatibility)
app.get('/api/user/profile', async (req, res) => {
  try {
    // Get session from Better Auth using cookies
    const session = await auth.api.getSession({ headers: req.headers });

    if (!session || !session.user) {
      return res.status(401).json({ error: 'Not authenticated' });
    }

    const pool = new Pool({
      connectionString: process.env.DATABASE_URL,
      max: 1
    });

    // Query user profile from database
    let result = await pool.query(
      'SELECT * FROM user_profiles WHERE "userId" = $1',
      [session.user.id]
    );

    // Auto-create profile for existing users (backward compatibility migration)
    if (result.rows.length === 0) {
      await pool.query(`
        INSERT INTO user_profiles ("userId", "createdAt", "updatedAt")
        VALUES ($1, NOW(), NOW())
        ON CONFLICT ("userId") DO NOTHING
      `, [session.user.id]);

      // Re-fetch the newly created profile
      result = await pool.query(
        'SELECT * FROM user_profiles WHERE "userId" = $1',
        [session.user.id]
      );
    }

    await pool.end();

    const profile = result.rows.length > 0 ? result.rows[0] : null;

    res.json({
      user: session.user,
      profile: {
        skill_level: profile?.skill_level || null,
        software_background: profile?.software_background || null,
        hardware_background: profile?.hardware_background || null,
        learning_goal: profile?.learning_goal || null
      }
    });
  } catch (error) {
    console.error('Profile fetch error:', error);
    res.status(500).json({ error: 'Failed to fetch profile' });
  }
});

// Save/Update user profile endpoint
app.post('/api/user/profile', async (req, res) => {
  try {
    // Get session from Better Auth using cookies
    const session = await auth.api.getSession({ headers: req.headers });

    if (!session || !session.user) {
      return res.status(401).json({ error: 'Not authenticated' });
    }

    const { skill_level, software_background, hardware_background, learning_goal } = req.body;

    const pool = new Pool({
      connectionString: process.env.DATABASE_URL,
      max: 1
    });

    // Upsert profile data
    await pool.query(`
      INSERT INTO user_profiles ("userId", skill_level, software_background, hardware_background, learning_goal, "createdAt", "updatedAt")
      VALUES ($1, $2, $3, $4, $5, NOW(), NOW())
      ON CONFLICT ("userId") DO UPDATE SET
        skill_level = COALESCE(EXCLUDED.skill_level, user_profiles.skill_level),
        software_background = COALESCE(EXCLUDED.software_background, user_profiles.software_background),
        hardware_background = COALESCE(EXCLUDED.hardware_background, user_profiles.hardware_background),
        learning_goal = COALESCE(EXCLUDED.learning_goal, user_profiles.learning_goal),
        "updatedAt" = NOW()
    `, [
      session.user.id,
      skill_level || null,
      software_background || null,
      hardware_background || null,
      learning_goal || null
    ]);

    await pool.end();

    res.json({ success: true, message: 'Profile saved' });
  } catch (error) {
    console.error('Profile save error:', error);
    res.status(500).json({ error: 'Failed to save profile' });
  }
});

// Personalization endpoint
app.post('/api/personalize', async (req, res) => {
  try {
    const { chapter_id, currentContent, user_metadata } = req.body;

    if (!chapter_id || !currentContent || !user_metadata) {
      return res.status(400).json({ error: 'Missing required fields: chapter_id, currentContent, and user_metadata' });
    }

    // Personalize the content based on user profile without adding profile data
    // This simulates AI-based personalization that adjusts content depth, examples, terminology, etc.
    let personalizedContent = currentContent;

    // Adjust content based on user's skill level and learning goal
    if (user_metadata.software_background) {
      // Simplify or enhance content based on user's background
      if (user_metadata.software_background.includes('beginner')) {
        // Add more explanations for beginners
        personalizedContent = personalizedContent.replace(/\b(concept|method|approach)\b/gi, '$1 (basic concept)');
      } else if (user_metadata.software_background.includes('advanced')) {
        // Add more technical depth for advanced users
        personalizedContent = personalizedContent.replace(/\b(example|concept|method)\b/gi, '$1 (advanced implementation)');
      }
    }

    if (user_metadata.learning_goal) {
      // Adjust focus based on learning goal
      if (user_metadata.learning_goal.toLowerCase().includes('ai') || user_metadata.learning_goal.toLowerCase().includes('ml')) {
        personalizedContent += '\n\n*This explanation is tailored for AI/ML learners.*';
      } else if (user_metadata.learning_goal.toLowerCase().includes('robotics')) {
        personalizedContent += '\n\n*This explanation is tailored for robotics enthusiasts.*';
      }
    }

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

// Example chat endpoint (placeholder - would need actual implementation)
app.post('/chat', async (req, res) => {
  try {
    const { message, selected_text, session_id } = req.body;

    // Placeholder response - in a real implementation, this would connect to your AI backend
    res.json({
      response: `This is a placeholder response for: ${message}`,
      source_chunks: [],
      session_id: session_id || 'default-session',
      citations: []
    });
  } catch (error) {
    res.status(500).json({ error: 'Chat processing failed' });
  }
});

// Endpoint to update user profile
app.put('/api/auth/profile', async (req, res) => {
  try {
    const { software_background, hardware_background, learning_goal } = req.body;

    // In a real implementation, this would update the user's profile in the database
    // For now, we'll just return a success response
    res.json({
      success: true,
      message: 'Profile updated successfully',
      user: {
        software_background,
        hardware_background,
        learning_goal
      }
    });
  } catch (error) {
    console.error('Profile update error:', error);
    res.status(500).json({ error: 'Profile update failed' });
  }
});

app.listen(port, () => {
  console.log(`API server running at https://fastapi-backend-for-book.vercel.app`);
  console.log(`Better Auth API available at https://fastapi-backend-for-book.vercel.app/api/auth`);
});