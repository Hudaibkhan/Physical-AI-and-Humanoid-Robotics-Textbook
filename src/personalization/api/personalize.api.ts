import { Context } from 'koa';
import { PersonalizationRequest, PersonalizationResponse } from '../user-chapter-state.entity';
import { UserChapterStateRepository } from '../../database/repositories/user-chapter-state.repository';
import { PersonalizedContentCacheRepository } from '../../database/repositories/personalized-content-cache.repository';
import { PersonalizationError } from '../../utils/error-handler.util';

const userChapterStateRepo = new UserChapterStateRepository();
const cacheRepo = new PersonalizedContentCacheRepository();

/**
 * Personalize chapter content API endpoint
 */
export async function personalizeContent(ctx: Context) {
  try {
    const { chapter_id, personalization_level, language } = ctx.request.body as PersonalizationRequest;

    // Extract user from context (should be set by auth middleware)
    const user = ctx.state.user;
    if (!user) {
      ctx.status = 401;
      ctx.body = {
        success: false,
        error: 'Authentication required'
      };
      return;
    }

    // Validate required fields
    if (!chapter_id) {
      ctx.status = 400;
      ctx.body = {
        success: false,
        error: 'Chapter ID is required'
      };
      return;
    }

    // Get user's metadata from their profile
    const userMetadata = {
      skill_level: user.skill_level || 'beginner',
      hardware_background: user.hardware_background || '',
      learning_goal: user.learning_goal || ''
    };

    // Check if we have cached content for this user and chapter
    let cachedContent = await cacheRepo.getByUserAndChapter(user.id, chapter_id);

    if (cachedContent) {
      // Return cached content if it exists
      ctx.status = 200;
      ctx.body = {
        success: true,
        chapter_id,
        personalized_content: cachedContent.content,
        metadata: {
          personalization_level: cachedContent.updated_at.toString(), // In real scenario, this would be the actual level
          language: language || 'en'
        }
      };
      return;
    }

    // In a real implementation, we would call the Python book_rag_agent with user metadata
    // For now, we'll create mock personalized content, but in a real app this would be:
    // 1. Call the FastAPI backend endpoint that uses the book_rag_agent
    // 2. Pass user metadata to influence the personalization
    // 3. Ensure personalized content maintains original citations and formatting
    console.log(`Personalizing chapter ${chapter_id} for user ${user.id} with metadata:`, userMetadata);

    // In a real implementation, make an API call to the Python backend
    const personalizedContent = await callPythonAgentForPersonalization(
      chapter_id,
      userMetadata,
      personalization_level || userMetadata.skill_level || 'beginner',
      language || 'en'
    );

    // Save personalization state
    await userChapterStateRepo.createOrUpdate({
      user_id: user.id,
      chapter_id,
      personalization_level: personalization_level || userMetadata.skill_level || 'beginner',
      language: language || 'en',
      updated_at: new Date()
    });

    // Cache the personalized content
    await cacheRepo.createOrUpdate({
      user_id: user.id,
      chapter_id,
      content: personalizedContent,
      updated_at: new Date()
    });

    const response: PersonalizationResponse = {
      success: true,
      chapter_id,
      personalized_content: personalizedContent,
      metadata: {
        personalization_level: personalization_level || userMetadata.skill_level || 'beginner',
        language: language || 'en'
      }
    };

    ctx.status = 200;
    ctx.body = response;
  } catch (error) {
    console.error('Personalization error:', error);
    ctx.status = 500;
    ctx.body = {
      success: false,
      error: error instanceof Error ? error.message : 'Personalization failed'
    };
  }
}

/**
 * Error handling middleware for unauthenticated requests
 */
export async function requireAuthentication(ctx: Context, next: () => Promise<any>) {
  // Extract user from context (should be set by auth middleware)
  const user = ctx.state.user;
  if (!user) {
    ctx.status = 401;
    ctx.body = {
      success: false,
      error: 'Authentication required'
    };
    return; // Don't call next() if not authenticated
  }

  await next(); // Continue to the next middleware/route handler
}

/**
 * Call the Python agent for personalization (mock implementation)
 * In a real implementation, this would make an API call to the FastAPI backend
 */
async function callPythonAgentForPersonalization(
  chapterId: string,
  userMetadata: any,
  personalizationLevel: string,
  language: string
): Promise<string> {
  // In a real implementation, this would be an API call to the Python backend:
  // const response = await fetch('http://localhost:8000/personalize-with-agent', {
  //   method: 'POST',
  //   headers: { 'Content-Type': 'application/json' },
  //   body: JSON.stringify({
  //     chapter_id: chapterId,
  //     user_metadata: userMetadata,
  //     personalization_level: personalizationLevel,
  //     language: language
  //   })
  // });
  //
  // if (!response.ok) {
  //   throw new Error(`Python agent call failed: ${response.statusText}`);
  // }
  //
  // const result = await response.json();
  // return result.personalized_content;

  // Mock implementation for demonstration
  return `
# Personalized Chapter Content

This content has been tailored to your background:

- **Skill Level**: ${userMetadata.skill_level}
- **Hardware Background**: ${userMetadata.hardware_background}
- **Learning Goal**: ${userMetadata.learning_goal}

The content below is adapted to match your experience level and interests.

## Original Chapter Content (Personalized)

In a real implementation, this would be the original chapter content that has been processed by the book_rag_agent with your user context. The agent would:

- Adapt complexity based on your skill level (${userMetadata.skill_level})
- Include relevant examples based on your hardware background (${userMetadata.hardware_background})
- Focus on topics aligned with your learning goal (${userMetadata.learning_goal})
- Maintain all original citations and formatting

*Personalized for your learning journey.*
  `.trim();
}

/**
 * Get personalization state for a specific chapter
 */
export async function getPersonalizationState(ctx: Context) {
  try {
    const { chapter_id } = ctx.params;

    // Extract user from context
    const user = ctx.state.user;
    if (!user) {
      ctx.status = 401;
      ctx.body = {
        success: false,
        error: 'Authentication required'
      };
      return;
    }

    if (!chapter_id) {
      ctx.status = 400;
      ctx.body = {
        success: false,
        error: 'Chapter ID is required'
      };
      return;
    }

    // Get personalization state from database
    const state = await userChapterStateRepo.getByUserAndChapter(user.id, chapter_id);

    if (!state) {
      ctx.status = 404;
      ctx.body = {
        success: false,
        error: 'No personalization state found for this chapter'
      };
      return;
    }

    ctx.status = 200;
    ctx.body = {
      success: true,
      state
    };
  } catch (error) {
    console.error('Get personalization state error:', error);
    ctx.status = 500;
    ctx.body = {
      success: false,
      error: error instanceof Error ? error.message : 'Failed to get personalization state'
    };
  }
}

/**
 * Update personalization state for a specific chapter
 */
export async function updatePersonalizationState(ctx: Context) {
  try {
    const { chapter_id } = ctx.params;
    const { personalization_level, language } = ctx.request.body;

    // Extract user from context
    const user = ctx.state.user;
    if (!user) {
      ctx.status = 401;
      ctx.body = {
        success: false,
        error: 'Authentication required'
      };
      return;
    }

    if (!chapter_id) {
      ctx.status = 400;
      ctx.body = {
        success: false,
        error: 'Chapter ID is required'
      };
      return;
    }

    // Update personalization state in database
    const newState = await userChapterStateRepo.createOrUpdate({
      user_id: user.id,
      chapter_id,
      personalization_level: personalization_level || 'beginner',
      language: language || 'en',
      updated_at: new Date()
    });

    ctx.status = 200;
    ctx.body = {
      success: true,
      state: newState
    };
  } catch (error) {
    console.error('Update personalization state error:', error);
    ctx.status = 500;
    ctx.body = {
      success: false,
      error: error instanceof Error ? error.message : 'Failed to update personalization state'
    };
  }
}