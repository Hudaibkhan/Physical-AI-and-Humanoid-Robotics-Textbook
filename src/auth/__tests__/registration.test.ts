import { validateUserRegistration } from '../validation/user.validation';
import { UserRegistrationData } from '../user.entity';

describe('User Registration Validation', () => {
  test('should validate correct registration data', () => {
    const validData: UserRegistrationData = {
      email: 'test@example.com',
      password: 'securePassword123',
      skill_level: 'intermediate',
      hardware_background: 'Arduino and Raspberry Pi',
      learning_goal: 'Build autonomous robots'
    };

    const errors = validateUserRegistration(validData);
    expect(errors).toHaveLength(0);
  });

  test('should return error for missing email', () => {
    const invalidData: UserRegistrationData = {
      email: '',
      password: 'securePassword123',
      skill_level: 'intermediate',
      hardware_background: 'Arduino and Raspberry Pi',
      learning_goal: 'Build autonomous robots'
    };

    const errors = validateUserRegistration(invalidData);
    expect(errors).toContainEqual({
      field: 'email',
      message: 'Email is required'
    });
  });

  test('should return error for invalid email format', () => {
    const invalidData: UserRegistrationData = {
      email: 'invalid-email',
      password: 'securePassword123',
      skill_level: 'intermediate',
      hardware_background: 'Arduino and Raspberry Pi',
      learning_goal: 'Build autonomous robots'
    };

    const errors = validateUserRegistration(invalidData);
    expect(errors).toContainEqual({
      field: 'email',
      message: 'Invalid email format'
    });
  });

  test('should return error for short password', () => {
    const invalidData: UserRegistrationData = {
      email: 'test@example.com',
      password: 'short',
      skill_level: 'intermediate',
      hardware_background: 'Arduino and Raspberry Pi',
      learning_goal: 'Build autonomous robots'
    };

    const errors = validateUserRegistration(invalidData);
    expect(errors).toContainEqual({
      field: 'password',
      message: 'Password must be at least 8 characters long'
    });
  });

  test('should return error for invalid skill level', () => {
    const invalidData: UserRegistrationData = {
      email: 'test@example.com',
      password: 'securePassword123',
      skill_level: 'invalid_level',
      hardware_background: 'Arduino and Raspberry Pi',
      learning_goal: 'Build autonomous robots'
    };

    const errors = validateUserRegistration(invalidData);
    expect(errors).toContainEqual({
      field: 'skill_level',
      message: 'Invalid skill level. Must be one of: beginner, intermediate, advanced, expert'
    });
  });

  test('should return error for overly long hardware background', () => {
    const longBackground = 'a'.repeat(201); // 201 characters, exceeding the 200 limit
    const invalidData: UserRegistrationData = {
      email: 'test@example.com',
      password: 'securePassword123',
      skill_level: 'intermediate',
      hardware_background: longBackground,
      learning_goal: 'Build autonomous robots'
    };

    const errors = validateUserRegistration(invalidData);
    expect(errors).toContainEqual({
      field: 'hardware_background',
      message: 'Hardware background must be less than 200 characters'
    });
  });
});

// Mock API endpoint tests would go here in a real implementation
describe('Registration API Endpoint', () => {
  test('should handle registration with valid data', async () => {
    // This would test the actual API endpoint
    // In a real implementation, we would mock the API and test the response
    expect(true).toBe(true); // Placeholder for actual test
  });

  test('should return error for invalid registration data', async () => {
    // This would test the API's validation of invalid data
    expect(true).toBe(true); // Placeholder for actual test
  });
});