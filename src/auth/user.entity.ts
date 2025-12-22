/**
 * User entity representing a registered user with authentication credentials and background metadata
 */
export interface User {
  id: string;                    // Unique user identifier from Better Auth
  email: string;                 // User's email address (required)
  skill_level?: string;          // User's skill level (metadata)
  hardware_background?: string;  // User's hardware background (metadata)
  learning_goal?: string;        // User's learning goal (metadata)
  created_at: Date;              // Account creation timestamp
  updated_at: Date;              // Last update timestamp
}

/**
 * User registration data transfer object
 */
export interface UserRegistrationData {
  email: string;
  password: string;
  skill_level?: string;
  hardware_background?: string;
  learning_goal?: string;
}

/**
 * User login data transfer object
 */
export interface UserLoginData {
  email: string;
  password: string;
}

/**
 * User metadata for personalization
 */
export interface UserMetadata {
  skill_level?: string;
  hardware_background?: string;
  learning_goal?: string;
}