import { Context, Next } from 'koa'; // Using Koa as an example, but could be adapted for other frameworks
import { auth } from '../auth/auth.config';

/**
 * Authentication middleware to validate JWT tokens from Better Auth
 */
export async function authMiddleware(ctx: Context, next: Next) {
  try {
    // Extract token from Authorization header
    const authHeader = ctx.headers.authorization;
    if (!authHeader || !authHeader.startsWith('Bearer ')) {
      ctx.status = 401;
      ctx.body = { error: 'Authorization header missing or invalid' };
      return;
    }

    const token = authHeader.substring(7); // Remove 'Bearer ' prefix

    // Verify the token using Better Auth
    const session = await auth.getSession({
      sessionToken: token
    });

    if (!session || !session.user) {
      ctx.status = 401;
      ctx.body = { error: 'Invalid or expired session token' };
      return;
    }

    // Add user info to context for use in subsequent middleware/handlers
    ctx.state.user = {
      id: session.user.id,
      email: session.user.email,
      name: session.user.name,
      skill_level: session.user.$?.skill_level || "",
      hardware_background: session.user.$?.hardware_background || "",
      learning_goal: session.user.$?.learning_goal || "",
    };
    ctx.state.session = session;

    // Continue to next middleware
    await next();
  } catch (error) {
    console.error('Authentication middleware error:', error);
    ctx.status = 500;
    ctx.body = { error: 'Internal server error during authentication' };
  }
}

/**
 * Optional: Middleware to check if user is authenticated
 */
export async function requireAuth(ctx: Context, next: Next) {
  if (!ctx.state.user) {
    ctx.status = 401;
    ctx.body = { error: 'Authentication required' };
    return;
  }
  await next();
}

/**
 * Optional: Middleware to check specific user permissions/roles
 */
export async function requireRole(roles: string[]) {
  return async (ctx: Context, next: Next) => {
    if (!ctx.state.user) {
      ctx.status = 401;
      ctx.body = { error: 'Authentication required' };
      return;
    }

    // In a real implementation, you would check user roles/permissions
    // For now, we'll just allow all authenticated users
    await next();
  };
}