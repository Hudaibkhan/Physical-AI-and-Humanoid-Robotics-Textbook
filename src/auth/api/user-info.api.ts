import { Context } from 'koa';
import { auth } from '../auth.config';

/**
 * User info API endpoint that validates token and returns user information
 */
export async function getUserInfo(ctx: Context) {
  try {
    // Extract token from Authorization header
    const authHeader = ctx.headers.authorization;
    if (!authHeader || !authHeader.startsWith('Bearer ')) {
      ctx.status = 401;
      ctx.body = {
        success: false,
        error: 'Authorization header missing or invalid'
      };
      return;
    }

    const token = authHeader.substring(7); // Remove 'Bearer ' prefix

    // Verify the token using Better Auth
    const session = await auth.getSession({
      sessionToken: token
    });

    if (!session || !session.user) {
      ctx.status = 401;
      ctx.body = {
        success: false,
        error: 'Invalid or expired session token'
      };
      return;
    }

    // Return user information
    ctx.status = 200;
    ctx.body = {
      success: true,
      user: {
        id: session.user.id,
        email: session.user.email,
        name: session.user.name,
        skill_level: session.user.$?.skill_level || "",
        hardware_background: session.user.$?.hardware_background || "",
        learning_goal: session.user.$?.learning_goal || "",
        created_at: session.user.createdAt,
        updated_at: session.user.updatedAt,
      }
    };
  } catch (error) {
    console.error('Get user info error:', error);
    ctx.status = 500;
    ctx.body = {
      success: false,
      error: error instanceof Error ? error.message : 'Failed to get user info'
    };
  }
}