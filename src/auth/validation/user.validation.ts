import { UserRegistrationData } from '../user.entity';

export interface ValidationError {
  field: string;
  message: string;
}

/**
 * Validates user registration data
 */
export function validateUserRegistration(data: UserRegistrationData): ValidationError[] {
  const errors: ValidationError[] = [];

  // Email validation
  if (!data.email) {
    errors.push({ field: 'email', message: 'Email is required' });
  } else {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(data.email)) {
      errors.push({ field: 'email', message: 'Invalid email format' });
    }
  }

  // Password validation
  if (!data.password) {
    errors.push({ field: 'password', message: 'Password is required' });
  } else if (data.password.length < 8) {
    errors.push({ field: 'password', message: 'Password must be at least 8 characters long' });
  }

  // Skill level validation (if provided)
  if (data.skill_level && !['beginner', 'intermediate', 'advanced', 'expert'].includes(data.skill_level)) {
    errors.push({ field: 'skill_level', message: 'Invalid skill level. Must be one of: beginner, intermediate, advanced, expert' });
  }

  // Hardware background validation (length check if provided)
  if (data.hardware_background && data.hardware_background.length > 200) {
    errors.push({ field: 'hardware_background', message: 'Hardware background must be less than 200 characters' });
  }

  // Learning goal validation (length check if provided)
  if (data.learning_goal && data.learning_goal.length > 200) {
    errors.push({ field: 'learning_goal', message: 'Learning goal must be less than 200 characters' });
  }

  return errors;
}

/**
 * Validates user metadata fields specifically
 */
export function validateUserMetadata(
  skill_level?: string,
  hardware_background?: string,
  learning_goal?: string
): ValidationError[] {
  const errors: ValidationError[] = [];

  if (skill_level && !['beginner', 'intermediate', 'advanced', 'expert'].includes(skill_level)) {
    errors.push({ field: 'skill_level', message: 'Invalid skill level. Must be one of: beginner, intermediate, advanced, expert' });
  }

  if (hardware_background && hardware_background.length > 200) {
    errors.push({ field: 'hardware_background', message: 'Hardware background must be less than 200 characters' });
  }

  if (learning_goal && learning_goal.length > 200) {
    errors.push({ field: 'learning_goal', message: 'Learning goal must be less than 200 characters' });
  }

  return errors;
}

/**
 * Sanitizes user input to prevent injection attacks
 */
export function sanitizeUserData(data: any): any {
  return {
    ...data,
    email: data.email ? data.email.trim().toLowerCase() : undefined,
    skill_level: data.skill_level ? data.skill_level.trim() : undefined,
    hardware_background: data.hardware_background ? data.hardware_background.trim() : undefined,
    learning_goal: data.learning_goal ? data.learning_goal.trim() : undefined,
  };
}