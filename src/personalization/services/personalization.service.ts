import { book_rag_agent } from '../../../../fastapi_app/agent'; // This is Python, so we'll need to call it differently
import { User } from '../../auth/user.entity';
import { PersonalizationRequest } from '../user-chapter-state.entity';

/**
 * Personalization Service that integrates with the existing book_rag_agent
 * Note: In a real implementation, this would call the Python agent via API
 */
export class PersonalizationService {

  /**
   * Generate personalized content for a chapter based on user metadata
   */
  async generatePersonalizedContent(
    chapterId: string,
    user: User,
    personalizationRequest?: PersonalizationRequest
  ): Promise<string> {
    // In a real implementation, this would:
    // 1. Call the existing book_rag_agent with user context
    // 2. Pass user metadata (skill_level, hardware_background, learning_goal) to influence the response
    // 3. Ensure the personalized content maintains original citations and formatting

    // For now, we'll create a mock implementation that simulates calling the Python agent
    console.log(`Generating personalized content for chapter ${chapterId} for user ${user.id}`);
    console.log(`User metadata:`, {
      skill_level: user.skill_level,
      hardware_background: user.hardware_background,
      learning_goal: user.learning_goal
    });

    // Simulate calling the book_rag_agent with user context
    // In a real implementation, this would be an API call to the FastAPI backend
    const personalizedContent = await this.callAgentWithUserContext(
      chapterId,
      user,
      personalizationRequest
    );

    return personalizedContent;
  }

  /**
   * Helper method to simulate calling the book_rag_agent with user context
   * In a real implementation, this would make an API call to the Python backend
   */
  private async callAgentWithUserContext(
    chapterId: string,
    user: User,
    request?: PersonalizationRequest
  ): Promise<string> {
    // This is where the actual integration with the Python book_rag_agent would happen
    // The agent would receive the chapter content along with user metadata
    // and generate personalized content based on the user's background

    // For now, return mock personalized content
    const originalContent = await this.getOriginalChapterContent(chapterId);

    // Create personalized content based on user metadata
    const skillLevel = user.skill_level || 'beginner';
    const hardwareBackground = user.hardware_background || 'general robotics';
    const learningGoal = user.learning_goal || 'learning robotics concepts';

    // In a real implementation, the agent would adapt:
    // - Complexity based on skill level
    // - Examples based on hardware background
    // - Focus areas based on learning goal
    const personalizedContent = `
# Personalized Chapter Content for ${user.email}

This content has been adapted to your background:

**Skill Level**: ${skillLevel}
**Hardware Background**: ${hardwareBackground}
**Learning Goal**: ${learningGoal}

---

${originalContent}

---

*This content has been personalized based on your profile. Examples and explanations have been adapted to match your experience level and interests.*
    `.trim();

    return personalizedContent;
  }

  /**
   * Get original chapter content (this would typically come from the book content system)
   */
  private async getOriginalChapterContent(chapterId: string): Promise<string> {
    // In a real implementation, this would fetch the original chapter content
    // from the book content system (e.g., from markdown files, database, etc.)
    return `
## Original Chapter Content

This is the original content of chapter ${chapterId}.

The actual chapter content would be retrieved from the book content system,
then processed by the book_rag_agent along with user metadata to generate
personalized content that maintains the original citations and formatting.

The agent ensures that:
- All original citations are preserved
- Content remains grounded in the original text
- Personalization adapts complexity, examples, and focus areas based on user background
    `.trim();
  }

  /**
   * In a real implementation, this would make an API call to the Python backend
   * to use the actual book_rag_agent with user context
   */
  async callPythonAgentAPI(
    chapterContent: string,
    user: User,
    request?: PersonalizationRequest
  ): Promise<string> {
    // Get auth token from localStorage if available
    const authToken = localStorage.getItem('authToken');

    // Get RAG backend URL from environment configuration
    const ENV_CONFIG = await import('../../config/env.config');
    const RAG_BACKEND_URL = ENV_CONFIG.default.RAG_BACKEND_URL;

    // Make API call to the Python backend for personalization
    const response = await fetch(`${RAG_BACKEND_URL}/personalize-agent`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...(authToken && { 'Authorization': `Bearer ${authToken}` })
      },
      body: JSON.stringify({
        chapter_content: chapterContent,
        user_metadata: {
          skill_level: user.skill_level,
          hardware_background: user.hardware_background,
          learning_goal: user.learning_goal,
        },
        personalization_level: request?.personalization_level || 'intermediate',
        language: request?.language || 'en',
      }),
    });

    if (!response.ok) {
      throw new Error(`Agent API call failed: ${response.statusText}`);
    }

    const result = await response.json();
    return result.personalized_content;
  }
}