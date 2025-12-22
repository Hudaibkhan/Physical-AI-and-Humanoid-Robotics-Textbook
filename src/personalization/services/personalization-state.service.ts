import { UserChapterStateRepository } from '../../database/repositories/user-chapter-state.repository';
import { UserChapterState } from '../user-chapter-state.entity';
import { User } from '../../auth/user.entity';

const userChapterStateRepo = new UserChapterStateRepository();

/**
 * Service for managing personalization state persistence
 */
export class PersonalizationStateService {

  /**
   * Get personalization state for a specific user and chapter
   */
  async getPersonalizationState(userId: string, chapterId: string): Promise<UserChapterState | null> {
    return await userChapterStateRepo.getByUserAndChapter(userId, chapterId);
  }

  /**
   * Save or update personalization state for a specific user and chapter
   */
  async savePersonalizationState(personalizationState: UserChapterState): Promise<UserChapterState> {
    return await userChapterStateRepo.createOrUpdate(personalizationState);
  }

  /**
   * Get all personalization states for a user
   */
  async getUserPersonalizationStates(userId: string): Promise<UserChapterState[]> {
    return await userChapterStateRepo.getByUser(userId);
  }

  /**
   * Apply existing personalization settings when user revisits a chapter
   */
  async applyExistingPersonalization(
    user: User,
    chapterId: string,
    currentPersonalization: string
  ): Promise<{ hasExisting: boolean; existingState: UserChapterState | null; result: string }> {
    // Get existing personalization state
    const existingState = await this.getPersonalizationState(user.id, chapterId);

    if (existingState) {
      // If there's existing personalization state, we can enhance the current personalization
      // with the saved settings
      const enhancedPersonalization = this.enhanceWithSavedSettings(
        currentPersonalization,
        existingState,
        user
      );

      return {
        hasExisting: true,
        existingState,
        result: enhancedPersonalization
      };
    } else {
      // No existing personalization, return the current one as is
      return {
        hasExisting: false,
        existingState: null,
        result: currentPersonalization
      };
    }
  }

  /**
   * Enhance personalization content with saved settings
   */
  private enhanceWithSavedSettings(
    currentPersonalization: string,
    savedState: UserChapterState,
    user: User
  ): string {
    // In a real implementation, this would merge the current personalization
    // with the saved preferences to maintain consistency
    return `
<!-- Enhanced with saved preferences -->
${currentPersonalization}

---
*Last personalized: ${savedState.updated_at.toISOString()}*
*Saved settings: Level: ${savedState.personalization_level}, Language: ${savedState.language}*
    `.trim();
  }

  /**
   * Update personalization state when user modifies settings
   */
  async updatePersonalizationState(
    userId: string,
    chapterId: string,
    personalizationLevel?: string,
    language?: string
  ): Promise<UserChapterState> {
    const currentState = await this.getPersonalizationState(userId, chapterId);

    const newState: UserChapterState = {
      user_id: userId,
      chapter_id: chapterId,
      personalization_level: personalizationLevel || currentState?.personalization_level || 'beginner',
      language: language || currentState?.language || 'en',
      updated_at: new Date()
    };

    return await this.savePersonalizationState(newState);
  }
}