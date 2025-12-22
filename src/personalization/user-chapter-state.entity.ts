/**
 * UserChapterState entity storing the personalization state for a specific user-chapter combination
 */
export interface UserChapterState {
  user_id: string;                   // Reference to User ID (foreign key)
  chapter_id: string;                // Reference to Chapter ID
  personalization_level: string;     // Level of personalization applied (e.g., beginner, intermediate, advanced)
  language: string;                  // Language preference for personalization
  updated_at: Date;                  // Timestamp of last personalization update
}

/**
 * PersonalizedContentCache entity (Optional) - cached version of personalized content for improved performance
 */
export interface PersonalizedContentCache {
  user_id: string;                   // Reference to User ID
  chapter_id: string;                // Reference to Chapter ID
  content: string;                   // Personalized content cache
  updated_at: Date;                  // Timestamp of cache update
}

/**
 * Personalization request data transfer object
 */
export interface PersonalizationRequest {
  chapter_id: string;
  personalization_level?: string;
  language?: string;
}

/**
 * Personalization response data transfer object
 */
export interface PersonalizationResponse {
  success: boolean;
  chapter_id: string;
  personalized_content: string;
  metadata: {
    personalization_level: string;
    language: string;
  };
}