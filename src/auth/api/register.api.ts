import { Context } from 'koa';
import { auth } from '../auth.config';
import { UserRegistrationData } from '../user.entity';
import { AuthError } from '../../utils/error-handler.util';

/**
 * Registration API endpoint that creates a new user with metadata
 */
export async function registerUser(ctx: Context) {
  try {
    const { email, password, skill_level, hardware_background, learning_goal } = ctx.request.body as UserRegistrationData;

    // Validate required fields
    if (!email || !password) {
      ctx.status = 400;
      ctx.body = {
        success: false,
        error: 'Email and password are required'
      };
      return;
    }

    // Validate email format
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      ctx.status = 400;
      ctx.body = {
        success: false,
        error: 'Invalid email format'
      };
      return;
    }

    // Create user with Better Auth, including custom metadata fields
    const user = await auth.createUser({
      email,
      password,
      name: email.split('@')[0], // Use part of email as name
      // Custom metadata goes in the `$` property
      $: {
        skill_level,
        hardware_background,
        learning_goal
      }
    });

    // Response with user data
    const responseUser = {
      id: user.id,
      email: user.email,
      skill_level: user.$?.skill_level || "",
      hardware_background: user.$?.hardware_background || "",
      learning_goal: user.$?.learning_goal || "",
      created_at: user.createdAt,
      updated_at: user.updatedAt,
    };

    ctx.status = 200;
    ctx.body = {
      success: true,
      user: mockUser,
      message: 'User registered successfully'
    };
  } catch (error) {
    console.error('Registration error:', error);
    ctx.status = 500;
    ctx.body = {
      success: false,
      error: error instanceof Error ? error.message : 'Registration failed'
    };
  }
}

/**
 * Login API endpoint
 */
export async function loginUser(ctx: Context) {
  try {
    const { email, password } = ctx.request.body;

    if (!email || !password) {
      ctx.status = 400;
      ctx.body = {
        success: false,
        error: 'Email and password are required'
      };
      return;
    }

    // Sign in user with Better Auth
    const session = await auth.signIn.email({
      email,
      password,
    });

    if (!session) {
      ctx.status = 401;
      ctx.body = {
        success: false,
        error: 'Invalid email or password'
      };
      return;
    }

    // Response with session token and user data
    ctx.status = 200;
    ctx.body = {
      success: true,
      token: session.token,
      user: {
        id: session.user.id,
        email: session.user.email,
        skill_level: session.user.$?.skill_level || "",
        hardware_background: session.user.$?.hardware_background || "",
        learning_goal: session.user.$?.learning_goal || "",
      },
      message: 'Login successful'
    };
  } catch (error) {
    console.error('Login error:', error);
    ctx.status = 500;
    ctx.body = {
      success: false,
      error: error instanceof Error ? error.message : 'Login failed'
    };
  }
}