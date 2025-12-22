/**
 * Error types for the authentication and personalization system
 */
export class AuthError extends Error {
  public statusCode: number;
  public isOperational: boolean;

  constructor(message: string, statusCode: number = 401) {
    super(message);
    this.statusCode = statusCode;
    this.isOperational = true; // Mark as operational error (not a bug)

    // Set the prototype explicitly for proper instanceof checks
    Object.setPrototypeOf(this, AuthError.prototype);
  }
}

export class PersonalizationError extends Error {
  public statusCode: number;
  public isOperational: boolean;

  constructor(message: string, statusCode: number = 400) {
    super(message);
    this.statusCode = statusCode;
    this.isOperational = true;

    Object.setPrototypeOf(this, PersonalizationError.prototype);
  }
}

export class DatabaseError extends Error {
  public statusCode: number;
  public isOperational: boolean;

  constructor(message: string, originalError?: Error) {
    const fullMessage = originalError ? `${message}: ${originalError.message}` : message;
    super(fullMessage);
    this.statusCode = 500;
    this.isOperational = false; // Database errors might indicate bugs

    Object.setPrototypeOf(this, DatabaseError.prototype);
  }
}

/**
 * Generic error handler with logging
 */
export function handleError(error: Error, context: string = 'unknown'): void {
  // Log the error with context
  console.error(`[${new Date().toISOString()}] ERROR in ${context}:`, {
    message: error.message,
    stack: error.stack,
    name: error.name,
  });

  // Additional error reporting could go here
  // e.g., sending to error tracking service
}

/**
 * Success response formatter
 */
export function createSuccessResponse<T>(data: T, message?: string): { success: boolean; message?: string; data: T } {
  return {
    success: true,
    message,
    data,
  };
}

/**
 * Error response formatter
 */
export function createErrorResponse(error: Error, statusCode?: number): { success: boolean; error: string; statusCode: number } {
  return {
    success: false,
    error: error.message,
    statusCode: statusCode || (error as any).statusCode || 500,
  };
}

/**
 * Log information with context
 */
export function logInfo(context: string, message: string, data?: any): void {
  console.log(`[${new Date().toISOString()}] INFO ${context}: ${message}`, data ? JSON.stringify(data) : '');
}

/**
 * Log warnings with context
 */
export function logWarning(context: string, message: string, data?: any): void {
  console.warn(`[${new Date().toISOString()}] WARN ${context}: ${message}`, data ? JSON.stringify(data) : '');
}